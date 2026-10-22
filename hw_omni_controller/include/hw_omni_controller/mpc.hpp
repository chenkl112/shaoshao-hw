#pragma once
#include "model.hpp"
#include "fddp_solver.hpp"
#include <string>

namespace fddp {
template<> struct Dims<hw_omni::Problem> {
  static constexpr int NX=hw_omni::NX, NU=hw_omni::NU, N=hw_omni::N;
};
}
namespace hw_omni {
struct Solution {
  bool valid=false;
  std::string reason;
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
    if((b.lower.array()>b.upper.array()).any()){Solution failed;failed.reason="Inconsistent speed/acceleration/jerk bounds";return failed;}
    solver.us[k]=solver.xs[k].segment<2>(4).cwiseMax(b.lower).cwiseMin(b.upper);
    solver.xs[k+1]=problem.dynamics(k,solver.xs[k],solver.us[k]);
  }
  const auto baseline_states=solver.xs;
  const auto baseline_commands=solver.us;
  fddp::SolverOptions options;options.max_iters=problem.settings.iterations;options.tol_grad=1e-5;
  auto result=solver.solve(problem,options);
  Solution solution;
  solution.states=solver.xs;solution.commands=solver.us;solution.cost=result.cost;
  solution.valid=result.feasible&&std::isfinite(result.cost)&&problem.safe(solver.xs,solver.us);
  // FDDP's QP enforces axis bounds. The vector speed disk and swept collision
  // constraints are checked on a complete rollout, including future commands.
  // Backtrack the whole trajectory toward the cold start if an optimized
  // future command exceeds those constraints. Never clip only today's output:
  // that would invalidate the response and jerk predictions used by the MPC.
  if(result.feasible&&std::isfinite(result.cost)&&!solution.valid){
    for(double alpha=.5;alpha>=1./1024;alpha*=.5){
      solution.states[0]=initial;bool bounded=true;double cost=0;
      for(int k=0;k<N;++k){
        solution.commands[k]=baseline_commands[k]+alpha*(solver.us[k]-baseline_commands[k]);
        auto bounds=problem.control_bounds(k,solution.states[k]);
        if((solution.commands[k].array()<bounds.lower.array()-1e-7).any()||
           (solution.commands[k].array()>bounds.upper.array()+1e-7).any()){bounded=false;break;}
        cost+=problem.running_cost(k,solution.states[k],solution.commands[k]);
        solution.states[k+1]=problem.dynamics(k,solution.states[k],solution.commands[k]);
      }
      if(bounded&&problem.safe(solution.states,solution.commands)){
        solution.cost=cost+problem.terminal_cost(solution.states[N]);
        solution.valid=std::isfinite(solution.cost);if(solution.valid)break;
      }
    }
    if(!solution.valid&&problem.safe(baseline_states,baseline_commands)){
      solution.states=baseline_states;solution.commands=baseline_commands;solution.cost=0;
      for(int k=0;k<N;++k)solution.cost+=problem.running_cost(k,solution.states[k],solution.commands[k]);
      solution.cost+=problem.terminal_cost(solution.states[N]);solution.valid=std::isfinite(solution.cost);
    }
  }
  if(!result.feasible)solution.reason="FDDP rollout infeasible";
  else if(!std::isfinite(result.cost))solution.reason="Nonfinite FDDP cost";
  else if(!solution.valid){
    solution.reason="Predicted rollout collision";
    for(const auto& state:solver.xs)if(problem.grid&&problem.grid->at(state.head<2>())>=253){solution.reason="Predicted static/unknown costmap collision";break;}
    for(const auto& command:solver.us)if(command.norm()>problem.settings.speed+1e-7){solution.reason="Predicted vector speed limit exceeded";break;}
  }
  if(solution.valid)solution.command=solution.commands[0];
  return solution;
}
}
