#pragma once
#include <Eigen/Core>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace hw_obstacles {
struct FloatField {size_t offset=0;uint8_t datatype=7;};
inline std::vector<Eigen::Vector3d> decode_cloud(const std::vector<uint8_t>& data,size_t width,size_t height,
    size_t point_step,size_t row_step,bool big_endian,const std::array<FloatField,3>& fields){
  if(!width||!height||width>200000||height>200000||width*height>200000||!point_step||
     point_step>4096||row_step<width*point_step||row_step>data.size()||height>data.size()/row_step)
    throw std::invalid_argument("Invalid cloud strides or point budget");
  for(const auto& f:fields)if((f.datatype!=7&&f.datatype!=8)||f.offset>point_step||
      size_t(f.datatype==7?4:8)>point_step-f.offset)
    throw std::invalid_argument("XYZ floating fields required");
  std::vector<Eigen::Vector3d> result;result.reserve(width*height);
  for(size_t y=0;y<height;++y)for(size_t x=0;x<width;++x){
    Eigen::Vector3d point;
    for(int axis=0;axis<3;++axis){const auto& f=fields[axis];size_t size=f.datatype==7?4:8;uint64_t bits=0;
      size_t start=y*row_step+x*point_step+f.offset;
      for(size_t b=0;b<size;++b)bits|=uint64_t(data[start+b])<<(8*(big_endian?size-b-1:b));
      point[axis]=f.datatype==7?static_cast<double>(std::bit_cast<float>(static_cast<uint32_t>(bits))):std::bit_cast<double>(bits);
    }
    if(point.allFinite())result.push_back(point);
  }
  return result;
}
}
