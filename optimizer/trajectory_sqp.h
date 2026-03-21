#include "optimizer/trajectory_optimizer.h"

namespace drake {
namespace systems {
// Forward declaration to avoid polluting this namespace with systems:: stuff.
template <typename>
class Diagram;
}  // namespace systems
}  // namespace drake

namespace idto {
namespace optimizer {

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::Diagram;
using internal::PentaDiagonalMatrix;

template <typename T>
class TrajectorySQP : public TrajectoryOptimizer<T> {
 public:
  using TrajectoryOptimizer<T>::TrajectoryOptimizer;

};

}  // namespace optimizer
}  // namespace idto