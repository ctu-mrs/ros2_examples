#ifndef COMMON_HANDLERS_H
#define COMMON_HANDLERS_H

#include <boost/function.hpp>
#include <eigen3/Eigen/Eigen>

namespace example_plugin_manager
{

//}

// | ---- logical units of supplied variables and functions --- |

typedef boost::function<double(const Eigen::Vector3d& input)> vectorNorm_t;

struct VectorCalculator_t
{
  bool                                 enabled;
  example_plugin_manager::vectorNorm_t vectorNorm;
};

// | ---------------------------------------------------------- |

struct CommonHandlers_t
{
  std::shared_ptr<std::string> some_shared_object;
  VectorCalculator_t           vector_calculator;
};


}  // namespace example_plugin_manager

#endif  // COMMON_HANDLERS_H
