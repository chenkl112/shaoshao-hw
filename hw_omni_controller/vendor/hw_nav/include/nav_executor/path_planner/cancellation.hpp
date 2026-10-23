#pragma once
#include <functional>
#include <stdexcept>
namespace nav_executor {
inline thread_local std::function<bool()> planning_cancel_checker;
inline void check_planning_canceled(){if(planning_cancel_checker&&planning_cancel_checker())throw std::runtime_error("HW planning canceled");}
}
