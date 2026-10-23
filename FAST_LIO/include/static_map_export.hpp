#pragma once
#include "static_map_projection.hpp"
#include <png.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <unistd.h>

inline void export_static_grid(const std::filesystem::path& directory,const StaticMapProjection::Grid& grid){
    if(!grid.width||!grid.height||uint64_t(grid.width)*grid.height>16000000||
       grid.data.size()!=size_t(grid.width)*grid.height)throw std::runtime_error("Invalid exported map dimensions");
    bool known=false;std::vector<unsigned char> pixels(grid.data.size());
    for(unsigned y=0;y<grid.height;++y)for(unsigned x=0;x<grid.width;++x){
        int value=grid.data[size_t(y)*grid.width+x];known|=value>=0;
        pixels[size_t(grid.height-1-y)*grid.width+x]=value<0?205:value<50?254:0;
    }
    if(!known)throw std::runtime_error("Saved PCD has no projected known cells");
    auto write=[](const std::filesystem::path& path,const std::string& bytes){
        auto temporary=path.string()+".tmp-"+std::to_string(getpid());
        {std::ofstream file(temporary,std::ios::binary);file.write(bytes.data(),bytes.size());if(!file)throw std::runtime_error("Cannot write map export");}
        std::filesystem::rename(temporary,path);
    };
    png_image image{};image.version=PNG_IMAGE_VERSION;image.width=grid.width;image.height=grid.height;image.format=PNG_FORMAT_GRAY;
    png_alloc_size_t length=0;if(!png_image_write_to_memory(&image,nullptr,&length,0,pixels.data(),0,nullptr))throw std::runtime_error("Cannot size exported PNG");
    std::string bytes(length,'\0');if(!png_image_write_to_memory(&image,bytes.data(),&length,0,pixels.data(),0,nullptr))throw std::runtime_error("Cannot encode exported PNG");bytes.resize(length);
    auto ticks=std::chrono::system_clock::now().time_since_epoch().count();auto name="site-snapshot-"+std::to_string(ticks)+".png";
    write(directory/name,bytes);
    YAML::Node metadata;metadata["image"]=name;metadata["resolution"]=grid.resolution;
    metadata["origin"].push_back(grid.x);metadata["origin"].push_back(grid.y);metadata["origin"].push_back(0.);
    metadata["negate"]=0;metadata["occupied_thresh"]=.65;metadata["free_thresh"]=.196;metadata["mode"]="trinary";
    YAML::Emitter yaml;yaml.SetDoublePrecision(17);yaml<<metadata;
    if(!yaml.good())throw std::runtime_error("Cannot encode exported map YAML");write(directory/"site.yaml",std::string(yaml.c_str())+"\n");
}
