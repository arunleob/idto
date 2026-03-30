#include "optimizer/trajectory_cost.h"

namespace idto {
namespace optimizer {

QuadraticTrajectoryCost::ConstructCostHessianAndGradient(Eigen::VectorXd z_ref) {
  // Loop through each time step and construct the cost hessian and gradient
  for (int k = 0; k < num_steps(); ++k) {
    std::vector<std::vector<int>> stencil_k = GetStencil(k);
    std::vector<Eigen::MatrixXd> M_k = GetM(k, z_ref);
    for (int i = 0; i < stencil_k.size(); ++i) {
      for (int j = 0; j < stencil_k.size(); ++j) {
        cost_hessian_.block(stencil_k[i], stencil_k[j]) += M_k[i].transpose() * blocks_[k] * M_k[j];
      }
      cost_gradient_.segment(stencil_k[i], stencil_k[i].size()) += M_k[i].transpose() * blocks_[k] * (M_k[i] * z_ref.segment(stencil_k[i], stencil_k[i].size()) - z_ref.segment(stencil_k[i], stencil_k[i].size()));
    }
  }
}

/*
Configuration cost has the simplest stencil
*/
std::vector<std::vector<int>> ConfigurationCost::GetStencil(std::vector<std::vector<int>>& q_index,
                                                            std::vector<std::vector<int>>& u_index, int k) {

  return {q_index_[k]};
}  // namespace optimizer

std::vector<Eigen::MatrixXd> ConfigurationCost::GetM(int k, Eigen::VectorXd z_ref) {
  return {Eigen::MatrixXd::Identity(q_index_[k].size(), q_index_[k].size())};
}

}  // namespace optimizer

}  // namespace idto