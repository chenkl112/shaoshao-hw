#pragma once
#include "common.hpp"
#include <bit>
#include <map>

namespace hw_runtime {
inline Cloud read_pcd(const fs::path& path){
  std::ifstream file(path,std::ios::binary);if(!file)throw std::runtime_error("Cannot open static PCD");
  std::map<std::string,std::vector<std::string>> header;std::string line,format;size_t header_size=0;
  while(std::getline(file,line)){
    header_size+=line.size();if(header_size>65536)throw std::runtime_error("PCD header too large");
    std::istringstream in(line);std::string key,part;if(!(in>>key)||key[0]=='#')continue;
    while(in>>part)header[key].push_back(part);
    if(key=="DATA"){if(header[key].size()!=1)throw std::runtime_error("Invalid PCD DATA");format=header[key][0];break;}
  }
  if(format!="binary"&&format!="ascii")throw std::runtime_error("Only binary/ascii PCD is supported");
  auto fields=header["FIELDS"],sizes=header["SIZE"],types=header["TYPE"],counts=header["COUNT"];
  if(fields.empty()||fields.size()>64||sizes.size()!=fields.size()||types.size()!=fields.size()||header["POINTS"].size()!=1)throw std::runtime_error("Invalid PCD fields");
  if(counts.empty())counts.assign(fields.size(),"1");if(counts.size()!=fields.size())throw std::runtime_error("Invalid PCD counts");
  size_t points=std::stoull(header["POINTS"][0]);if(!points||points>20000000)throw std::runtime_error("PCD point budget exceeded");
  std::array<size_t,3> offsets{},widths{},columns{};std::array<bool,3> found{};size_t step=0,total_columns=0;
  const std::array<std::string,3> names{"x","y","z"};
  for(size_t i=0;i<fields.size();++i){size_t size=std::stoull(sizes[i]),count=std::stoull(counts[i]);if(!count||count>64||!size||size>8)throw std::runtime_error("Invalid PCD field size");
    for(int a=0;a<3;++a)if(fields[i]==names[a]){if(types[i]!="F"||(size!=4&&size!=8)||count!=1)throw std::runtime_error("PCD XYZ must be scalar floats");offsets[a]=step;widths[a]=size;columns[a]=total_columns;found[a]=true;}
    step+=size*count;total_columns+=count;
  }
  if(!std::all_of(found.begin(),found.end(),[](bool v){return v;})||step>4096)throw std::runtime_error("PCD XYZ fields missing");
  Cloud result;result.reserve(points);std::vector<unsigned char> bytes(step);
  for(size_t i=0;i<points;++i){Point p;
    if(format=="binary"){
      if(!file.read(reinterpret_cast<char*>(bytes.data()),step))throw std::runtime_error("Truncated binary PCD");
      for(int a=0;a<3;++a){uint64_t bits=0;for(size_t b=0;b<widths[a];++b)bits|=uint64_t(bytes[offsets[a]+b])<<(8*b);p[a]=widths[a]==4?double(std::bit_cast<float>(uint32_t(bits))):std::bit_cast<double>(bits);}
    }else for(size_t column=0;column<total_columns;++column){double value;if(!(file>>value))throw std::runtime_error("Truncated ASCII PCD");for(int a=0;a<3;++a)if(column==columns[a])p[a]=value;}
    if(p.allFinite())result.push_back(p);
  }
  return result;
}
inline void save_pcd(const fs::path& path,const Cloud& points){
  std::ostringstream header;header<<"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "<<points.size()<<"\nHEIGHT 1\nPOINTS "<<points.size()<<"\nDATA binary\n";
  std::string data=header.str();for(const auto& p:points)for(int i=0;i<3;++i){uint32_t bits=std::bit_cast<uint32_t>(static_cast<float>(p[i]));for(int b=0;b<4;++b)data.push_back(static_cast<char>((bits>>(8*b))&255));}atomic_write(path,data);
}
}
