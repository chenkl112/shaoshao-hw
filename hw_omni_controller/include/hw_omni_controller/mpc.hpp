#pragma once
#include "model.hpp"
#include "fddp_solver.hpp"

namespace fddp {
template<> struct Dims<hw_omni::Problem> {
  static constexpr int NX=hw_omni::NX, NU=hw_omni::NU, N=hw_omni::N;
};
}
namespace hw_omni {
struct Solution {
  bool valid=false;
  U command=U::Zero();
  std::array<X,N+1> states;
  std::array<U,N> commands;
  double cost=0;
};
inline Solution solve(const Problem& problem,const X& initial) {
  problem.settings.validate();
  // Cold-start from actual state every cycle. No previous trajectory can retain
  // control authority across cancellation, localization loss or a new lease.
  fddp::Solver<Problem> solver;
  solver.xs[0]=initial;
  for(int k=0;k<N;++k){
    auto b=problem.control_bounds(k,solver.xs[k]);
    if((b.lower.array()>b.upper.array()).any())return {};
    solver.us[k]=solver.xs[k].segment<2>(4).cwiseMax(b.lower).cwiseMin(b.upper);
    solver.xs[k+1]=problem.dynamics(k,solver.xs[k],solver.us[k]);
  }
  fddp::SolverOptions options;options.max_iters=problem.settings.iterations;options.tol_grad=1e-5;
  auto result=solver.solve(problem,options);
  Solution solution;
  solution.states=solver.xs;solution.commands=solver.us;solution.cost=result.cost;
  solution.valid=result.feasible&&std::isfinite(result.cost)&&problem.safe(solver.xs,solver.us);
  if(solution.valid)solution.command=solver.us[0];
  return solution;
}
}
