#include "optimizer/trajectory_sqp.h"

namespace idto {
namespace optimizer {

template <typename T>
TrajectorySQP<T>::TrajectorySQP(const Diagram<T>* diagram,
                                const MultibodyPlant<T>* plant,
                                const ProblemDefinition& prob,
                                const Eigen::VectorXd& scaling,
                                const SolverParameters& params)
    : TrajectoryOptimizer<T>(diagram, plant, prob, params),
      nq_(plant->num_positions()),
      nv_(plant->num_velocities()),
      nu_(plant->num_velocities() - this->unactuated_dofs_.size()),
      scaling_(scaling),
      nc_dynamics_(plant->num_velocities() * this->num_steps()) {
  // Init MathematicalProgram (order determines indexing)
  q_sym_.resize(this->num_steps());
  u_sym_.resize(this->num_steps());
  for (int k = 0; k < this->num_steps(); ++k) {
    q_sym_[k] = prog_.NewContinuousVariables(nq_, "q_" + std::to_string(k + 1));
  }
  for (int k = 0; k < this->num_steps(); ++k) {
    u_sym_[k] = prog_.NewContinuousVariables(nu_, "u_" + std::to_string(k + 1));
  }
     
  // Create index helpers for constructing problem matrices
  q_index_.resize(this->num_steps());
  u_index_.resize(this->num_steps());
  for (int k = 0; k < this->num_steps(); ++k) {
    q_index_[k] = prog_.FindDecisionVariableIndices(q_sym_[k]);
    u_index_[k] = prog_.FindDecisionVariableIndices(u_sym_[k]);
  }

  // Init dynamics indices, residuals, and jacobian
  dynamics_index_.resize(this->num_steps());
  for (int k = 0; k < this->num_steps(); ++k) {
    dynamics_index_[k] = Eigen::VectorXi::LinSpaced(nv_, nv_*k, nv_*(k + 1) - 1);
  }
  dynamics_residual_ = Eigen::VectorXd::Zero(nc_dynamics_);
  dynamics_jacobian_ = Eigen::MatrixXd::Zero(nc_dynamics_, prog_.num_vars());

  // Fill constant terms of jacobian
  auto tau_scaling = (-scaling_(u_index_[0])).asDiagonal();
  for (int k = 0; k < this->num_steps(); ++k) {
    dynamics_jacobian_(dynamics_index_[k].segment(nv_ - nu_, nu_), u_index_[k]) = tau_scaling;
  }

}

}  // namespace optimizer
}  // namespace idto

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::idto::optimizer::TrajectorySQP)