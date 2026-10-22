#pragma once
#include "common.hpp"
#include <minizip/unzip.h>
#include <minizip/zip.h>
#include <bit>
#include <cstring>
#include <regex>

namespace hw_runtime {
struct Array {
  std::vector<size_t> shape;std::vector<double> data;
};
inline Array read_npz(const fs::path& path,const std::string& name){
  struct Archive{unzFile file=nullptr;~Archive(){if(file)unzClose(file);}} archive;
  archive.file=unzOpen64(path.c_str());if(!archive.file||unzLocateFile(archive.file,(name+".npy").c_str(),1)!=UNZ_OK)throw std::runtime_error("Missing NPZ array: "+name);
  unz_file_info64 info{};if(unzGetCurrentFileInfo64(archive.file,&info,nullptr,0,nullptr,0,nullptr,0)!=UNZ_OK||info.uncompressed_size>128*1024*1024)throw std::runtime_error("NPZ array exceeds budget");
  if(unzOpenCurrentFile(archive.file)!=UNZ_OK)throw std::runtime_error("Cannot open NPZ array");
  std::string bytes(static_cast<size_t>(info.uncompressed_size),'\0');size_t read=0;
  while(read<bytes.size()){int n=unzReadCurrentFile(archive.file,bytes.data()+read,static_cast<unsigned>(std::min(size_t(65536),bytes.size()-read)));if(n<=0)throw std::runtime_error("Truncated NPZ array");read+=static_cast<size_t>(n);}
  if(unzCloseCurrentFile(archive.file)!=UNZ_OK)throw std::runtime_error("NPZ checksum failed");
  if(bytes.size()<10||bytes.substr(0,6)!=std::string("\x93NUMPY",6))throw std::runtime_error("Invalid NPY magic");
  unsigned version=static_cast<unsigned char>(bytes[6]);size_t offset=version==1?10:version==2?12:0;
  if(!offset||bytes.size()<offset)throw std::runtime_error("Unsupported NPY version");
  uint32_t length=0;for(size_t i=0;i<offset-8;++i)length|=uint32_t(static_cast<unsigned char>(bytes[8+i]))<<(8*i);
  if(length>65536||offset+length>bytes.size())throw std::runtime_error("Invalid NPY header length");
  std::string header=bytes.substr(offset,length);std::smatch match;
  if(!std::regex_search(header,match,std::regex("'descr'\\s*:\\s*'([<>|=])([fiu])([48])'")))throw std::runtime_error("Unsupported NPY type");
  char endian=match[1].str()[0],kind=match[2].str()[0];size_t width=static_cast<size_t>(std::stoi(match[3].str()));
  if(endian=='>'||kind!='f')throw std::runtime_error("Only little-endian float NPZ arrays are supported");
  if(!std::regex_search(header,std::regex("'fortran_order'\\s*:\\s*False")))throw std::runtime_error("Fortran NPZ arrays are not supported");
  if(!std::regex_search(header,match,std::regex("'shape'\\s*:\\s*\\(([^)]*)\\)")))throw std::runtime_error("Missing NPY shape");
  Array array;std::string dimensions=match[1].str();std::regex number("[0-9]+");size_t count=1;
  for(auto it=std::sregex_iterator(dimensions.begin(),dimensions.end(),number);it!=std::sregex_iterator();++it){size_t dim=std::stoull(it->str());if(!dim||dim>10000000||count>16000000/dim)throw std::runtime_error("NPY shape exceeds budget");array.shape.push_back(dim);count*=dim;}
  if(array.shape.empty()||array.shape.size()>3||bytes.size()-offset-length!=count*width)throw std::runtime_error("NPY data size mismatch");
  array.data.reserve(count);const auto* p=reinterpret_cast<const unsigned char*>(bytes.data()+offset+length);
  for(size_t i=0;i<count;++i){uint64_t bits=0;for(size_t b=0;b<width;++b)bits|=uint64_t(p[i*width+b])<<(8*b);double value=width==4?double(std::bit_cast<float>(uint32_t(bits))):std::bit_cast<double>(bits);if(!std::isfinite(value))throw std::runtime_error("Nonfinite NPZ data");array.data.push_back(value);}
  return array;
}
inline void write_npz(const fs::path& path,const std::vector<std::pair<std::string,Array>>& arrays,bool single_precision=false){
  auto tmp=path.string()+".tmp-"+std::to_string(getpid());
  struct Archive{zipFile file=nullptr;~Archive(){if(file)zipClose(file,nullptr);}} archive;
  archive.file=zipOpen64(tmp.c_str(),APPEND_STATUS_CREATE);if(!archive.file)throw std::runtime_error("Cannot create NPZ");
  for(const auto& [name,array]:arrays){
    size_t count=1;for(auto dimension:array.shape){if(!dimension||dimension>16000000||count>16000000/dimension)throw std::invalid_argument("Invalid NPZ shape");count*=dimension;}
    if(array.shape.empty()||array.shape.size()>3||count!=array.data.size())throw std::invalid_argument("NPZ shape mismatch");
    std::ostringstream dimensions;for(size_t dim:array.shape)dimensions<<dim<<", ";
    bool f32=single_precision&&name=="points";size_t width=f32?4:8;
    std::string header="{'descr': '<f"+std::to_string(width)+"', 'fortran_order': False, 'shape': ("+dimensions.str()+"), }";
    header+=std::string((64-(10+header.size()+1)%64)%64,' ')+"\n";
    std::string bytes("\x93NUMPY\x01\x00",8);bytes.push_back(static_cast<char>(header.size()&255));bytes.push_back(static_cast<char>((header.size()>>8)&255));bytes+=header;
    for(double value:array.data){if(!std::isfinite(value))throw std::invalid_argument("Nonfinite NPZ value");uint64_t bits=f32?std::bit_cast<uint32_t>(static_cast<float>(value)):std::bit_cast<uint64_t>(value);for(size_t b=0;b<width;++b)bytes.push_back(static_cast<char>((bits>>(8*b))&255));}
    zip_fileinfo info{};if(zipOpenNewFileInZip64(archive.file,(name+".npy").c_str(),&info,nullptr,0,nullptr,0,nullptr,Z_DEFLATED,Z_DEFAULT_COMPRESSION,0)!=ZIP_OK)throw std::runtime_error("Cannot write NPZ member");
    if(zipWriteInFileInZip(archive.file,bytes.data(),static_cast<unsigned>(bytes.size()))!=ZIP_OK||zipCloseFileInZip(archive.file)!=ZIP_OK)throw std::runtime_error("NPZ write failed");
  }
  if(zipClose(archive.file,nullptr)!=ZIP_OK){archive.file=nullptr;throw std::runtime_error("NPZ finalization failed");}archive.file=nullptr;fs::rename(tmp,path);
}
inline Array pose_array(const Pose& pose){Array a{{4,4},{}};for(int i=0;i<4;++i)for(int j=0;j<4;++j)a.data.push_back(pose.matrix()(i,j));return a;}
inline Pose array_pose(const Array& value,size_t offset=0){
  if(offset+16>value.data.size())throw std::invalid_argument("Invalid pose array");Eigen::Matrix4d matrix;
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)matrix(i,j)=value.data[offset+i*4+j];
  if((matrix.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-8||std::abs(matrix.block<3,3>(0,0).determinant()-1)>.01||
     (matrix.block<3,3>(0,0).transpose()*matrix.block<3,3>(0,0)-Eigen::Matrix3d::Identity()).norm()>.01)throw std::invalid_argument("Invalid rigid pose");
  Pose out=Pose::Identity();out.matrix()=matrix;return out;
}
inline Array cloud_array(const Cloud& points){Array a{{points.size(),3},{}};for(const auto& p:points)for(int i=0;i<3;++i)a.data.push_back(p[i]);return a;}
inline Cloud array_cloud(const Array& a){if(a.shape.size()!=2||a.shape[1]!=3)throw std::invalid_argument("XYZ cloud array required");Cloud points;for(size_t i=0;i<a.data.size();i+=3)points.emplace_back(a.data[i],a.data[i+1],a.data[i+2]);return points;}
}
