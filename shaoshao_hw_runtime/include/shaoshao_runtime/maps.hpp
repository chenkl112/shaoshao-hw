#pragma once
#include "common.hpp"
#include <png.h>

namespace hw_runtime {
struct SavedMap {Json::Value info;fs::path image;};
inline SavedMap saved_map(const fs::path& directory){
  SavedMap out;out.info["available"]=false;out.info["message"]="暂无建图地图，完成建图并保存后会显示在这里。";
  if(!fs::is_regular_file(directory/"site.yaml"))return out;
  try{
    auto root=fs::canonical(directory),metadata=directory/"site.yaml";auto config=YAML::LoadFile(metadata.string());
    auto image=fs::weakly_canonical(root/config["image"].as<std::string>()),relative=image.lexically_relative(root);
    if(relative.empty()||relative.is_absolute()||*relative.begin()==".."||image.extension()!=".png"||!fs::is_regular_file(image))throw std::runtime_error("地图图像必须是地图目录内的 PNG");
    double resolution=config["resolution"].as<double>();Point origin=yaml_point(config["origin"]);if(!std::isfinite(resolution)||resolution<=0)throw std::runtime_error("地图分辨率无效");
    std::array<unsigned char,24> header{};std::ifstream f(image,std::ios::binary);if(!f.read(reinterpret_cast<char*>(header.data()),header.size()))throw std::runtime_error("地图 PNG 头无效");
    const std::array<unsigned char,8> signature{137,80,78,71,13,10,26,10};if(!std::equal(signature.begin(),signature.end(),header.begin())||std::string(reinterpret_cast<char*>(header.data()+12),4)!="IHDR")throw std::runtime_error("地图 PNG 格式无效");
    auto read32=[&](int offset){uint32_t value=0;for(int i=0;i<4;++i)value=(value<<8)|header[offset+i];return value;};auto width=read32(16),height=read32(20);if(!width||!height||uint64_t(width)*height>16000000)throw std::runtime_error("地图尺寸超出预算");
    auto ticks=[](const fs::path& p){return fs::last_write_time(p).time_since_epoch().count();};std::string revision=std::to_string(ticks(metadata))+"-"+std::to_string(ticks(image))+"-"+std::to_string(fs::file_size(image));
    out.info["available"]=true;out.info["frame"]="map";out.info["resolution"]=resolution;out.info["width"]=width;out.info["height"]=height;out.info["origin"]=point_json(origin);out.info["revision"]=revision;out.info["image_url"]="/api/map/image?revision="+revision;out.image=image;
  }catch(const std::exception& e){out.info["message"]=std::string("地图暂不可用：")+e.what();}return out;
}
inline bool save_grid(const fs::path& directory,uint32_t width,uint32_t height,double resolution,const Point& origin,const std::vector<int8_t>& values){
  if(!width||!height||uint64_t(width)*height>16000000||values.size()!=size_t(width)*height||!std::isfinite(resolution)||resolution<=0||!origin.allFinite())throw std::invalid_argument("Invalid map grid");
  bool known=false;std::vector<unsigned char> pixels(values.size());
  for(uint32_t y=0;y<height;++y)for(uint32_t x=0;x<width;++x){int value=values[size_t(y)*width+x];if(value<-1||value>100)throw std::invalid_argument("Invalid occupancy value");known|=value>=0;pixels[size_t(height-y-1)*width+x]=value<0?205:value<50?254:0;}
  if(!known)return false;png_image image{};image.version=PNG_IMAGE_VERSION;image.width=width;image.height=height;image.format=PNG_FORMAT_GRAY;
  png_alloc_size_t size=0;if(!png_image_write_to_memory(&image,nullptr,&size,0,pixels.data(),0,nullptr))throw std::runtime_error("Cannot size map PNG");std::string bytes(size,'\0');
  if(!png_image_write_to_memory(&image,bytes.data(),&size,0,pixels.data(),0,nullptr))throw std::runtime_error("Cannot encode map PNG");bytes.resize(size);
  auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();std::string filename="live-grid-"+std::to_string(ns)+".png";atomic_write(directory/filename,bytes);
  Json::Value metadata;metadata["image"]=filename;metadata["resolution"]=resolution;metadata["origin"]=point_json(origin);metadata["negate"]=0;metadata["occupied_thresh"]=.65;metadata["free_thresh"]=.196;metadata["mode"]="trinary";write_yaml(directory/"site.yaml",YAML::Load(json_text(metadata)));
  std::vector<fs::path> old;for(const auto& entry:fs::directory_iterator(directory))if(entry.path().filename().string().starts_with("live-grid-")&&entry.path().extension()==".png")old.push_back(entry.path());std::sort(old.begin(),old.end());for(size_t i=0;i+8<old.size();++i)fs::remove(old[i]);return true;
}
}
