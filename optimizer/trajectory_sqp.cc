#include "optimizer/trajectory_sqp.h"
#include <iostream>

namespace idto {
namespace optimizer {

TrajectorySQP::TrajectorySQP(const Diagram<double>* diagram,
                                const MultibodyPlant<double>* plant,
                                const ProblemDefinition& prob,
                                const Eigen::VectorXd& scaling,
                                const SolverParameters& params)
    : TrajectoryOptimizer<double>(diagram, plant, prob, params),
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

void TrajectorySQP::UpdateDynamicsResidual(const TrajectoryOptimizerState<double>& state) {
  // Compute tau
  this->EvalEqualityConstraintViolations(state);

  // Update residual
  double tau_scale = scaling_(u_index_[0])[0]; // TODO extract once
  for (int k = 0; k < this->num_steps(); ++k) {
    //TODO: improve efficiency
    Eigen::VectorXd tau_unact = state.tau()[k](this->unactuated_dofs());
    Eigen::VectorXd tau_act = state.tau()[k](this->actuated_dofs());

    dynamics_residual_(dynamics_index_[k].segment(0, nv_ - nu_)) = tau_unact/tau_scale;
    dynamics_residual_(dynamics_index_[k].segment(nv_ - nu_, nu_)) = tau_act;
  }
}

void TrajectorySQP::updateDynamicsJacobian(const TrajectoryOptimizerState<double>& state) {
  // Compute jacobian
  Eigen::MatrixXd jacobian = this->EvalTauJacobian(state);

  // Update jacobian
  double tau_scale = scaling_(u_index_[0])[0]; // TODO extract once
  for (int k = 0; k < this->num_steps(); ++k) {
    Eigen::MatrixXd knot_jac = jacobian.block(k*nv_, (k + 1)*nq_, nv_, nq_);
    dynamics_jacobian_(dynamics_index_[k].segment(0, nv_ - nu_), q_index_[k]) = knot_jac(this->unactuated_dofs(), Eigen::all) / tau_scale;
    dynamics_jacobian_(dynamics_index_[k].segment(nv_ - nu_, nu_), q_index_[k]) = knot_jac(this->actuated_dofs(), Eigen::all);
  }
}


}  // namespace optimizer
}  // namespace idto