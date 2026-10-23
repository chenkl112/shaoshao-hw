#pragma once
#include "model.hpp"
namespace hw_omni {
// Nav2 has already inflated lethal obstacles by the robot footprint. Unknown
// cells need a separate circular expansion for HW's recovery field/search.
inline std::vector<unsigned char> recovery_cost_cells(const Grid& grid,double radius){
  if(grid.width<=0||grid.height<=0||grid.cells.size()!=size_t(grid.width)*grid.height||!std::isfinite(grid.resolution)||grid.resolution<=0||!std::isfinite(radius)||radius<=0)
    throw std::invalid_argument("Invalid recovery grid");
  auto cells=grid.cells;std::vector<std::pair<int,int>> kernel;
  int pad=int(std::ceil(radius/grid.resolution+.5));
  for(int dy=-pad;dy<=pad;++dy)for(int dx=-pad;dx<=pad;++dx){
    double x=std::max(0.,std::abs(dx)-.5)*grid.resolution,y=std::max(0.,std::abs(dy)-.5)*grid.resolution;
    if(x*x+y*y<=radius*radius)kernel.emplace_back(dx,dy);
  }
  for(int y=0;y<grid.height;++y)for(int x=0;x<grid.width;++x){
    if(grid.cells[size_t(y)*grid.width+x]==255)for(auto [dx,dy]:kernel){int a=x+dx,b=y+dy;if(a>=0&&b>=0&&a<grid.width&&b<grid.height)cells[size_t(b)*grid.width+a]=255;}
    if(std::min({x+.5,y+.5,grid.width-x-.5,grid.height-y-.5})*grid.resolution<radius)cells[size_t(y)*grid.width+x]=255;
  }
  return cells;
}
}
