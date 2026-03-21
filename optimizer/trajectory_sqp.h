#include "optimizer/trajectory_optimizer.h"

#include <drake/common/eigen_types.h>
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

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
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using internal::PentaDiagonalMatrix;

template <typename T>
class TrajectorySQP : public TrajectoryOptimizer<T> {
 public:
  using TrajectoryOptimizer<T>::TrajectoryOptimizer;
  
  TrajectorySQP(const Diagram<T>* diagram, const MultibodyPlant<T>* plant,
                      const ProblemDefinition& prob,
                      const SolverParameters& params = SolverParameters{});

  // Get indices into decision variable vector
  std::vector<std::vector<int>> GetQIndex() const { return q_index_; }
  std::vector<std::vector<int>> GetUIndex() const { return u_index_; }

  // Get dynamics residual and jacobian
  Eigen::VectorXd GetDynamicsResidual() const { return dynamics_residual_; }
  Eigen::MatrixXd GetDynamicsJacobian() const { return dynamics_jacobian_; }

  // Instantiate mathematical program
  MathematicalProgram prog_; 

  // Problem dimensions
  const int nq_;
  const int nu_;

  // Symbolic variables (TODO: are these used?)
  std::vector<VectorXDecisionVariable> q_sym_;
  std::vector<VectorXDecisionVariable> u_sym_;

  // Indices into SQP decision vector
  std::vector<std::vector<int>> q_index_;
  std::vector<std::vector<int>> u_index_;

  // Dynamics jacobian and residual
  const int nc_dynamics_;
  Eigen::VectorXd dynamics_residual_;
  Eigen::MatrixXd dynamics_jacobian_;
};

}  // namespace optimizer
}  // namespace idto