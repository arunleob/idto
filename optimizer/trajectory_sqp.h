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

class TrajectorySQP : public TrajectoryOptimizer<double> {
 public:
  using TrajectoryOptimizer<double>::TrajectoryOptimizer;
  
  TrajectorySQP(const Diagram<double>* diagram, const MultibodyPlant<double>* plant,
                      const ProblemDefinition& prob,
                      const Eigen::VectorXd& scaling,
                      const SolverParameters& params = SolverParameters{});

  // Get indices into decision variable vector
  std::vector<std::vector<int>> GetQIndex() const { return q_index_; }
  std::vector<std::vector<int>> GetUIndex() const { return u_index_; }

  // Get dynamics residual and jacobian
  Eigen::VectorXd GetDynamicsResidual() const { return dynamics_residual_; }
  Eigen::MatrixXd GetDynamicsJacobian() const { return dynamics_jacobian_; }

  // Update the dynamics constraint residual in place
  void UpdateDynamicsResidual(const TrajectoryOptimizerState<double>& state, const Eigen::VectorXd& z);

  // Update the dynamics constraint jacobian in place
  void updateDynamicsJacobian(const TrajectoryOptimizerState<double>& state);

  // Instantiate mathematical program
  MathematicalProgram prog_; 

  // Problem dimensions
  const int nq_;
  const int nv_;
  const int nu_;

  // Problem scaling
  const Eigen::VectorXd scaling_;

  // Symbolic variables (TODO: are these used?)
  std::vector<VectorXDecisionVariable> q_sym_;
  std::vector<VectorXDecisionVariable> u_sym_;

  // Indices into SQP decision vector
  std::vector<std::vector<int>> q_index_;
  std::vector<std::vector<int>> u_index_;

  // Dynamics jacobian and residual
  const int nc_dynamics_;
  std::vector<Eigen::VectorXi> dynamics_index_;
  Eigen::VectorXd dynamics_residual_;
  Eigen::MatrixXd dynamics_jacobian_;
};

}  // namespace optimizer
}  // namespace idto