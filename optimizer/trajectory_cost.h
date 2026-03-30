#include <drake/common/eigen_types.h>
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace idto {
namespace optimizer {

/*
  An abstract class for representing quadratic trajectory costs of the form
  
    L(y_k) = 1/2 y_k'*Q*y_k + q'*y_k + c

  where y_k is constructed from configurations and controls (primary decision variables).
  Given the stacked decision variable vector z, this is written as 

    y_k = sum_i M[k, i]*z[stencil[k, i]]
  
  where M[k, i] are matrices, and stencil[k, i] is a vector of vector indices into z. Each
  instantiation defines M and stencil, and this class gives functions to populate the hessian
  and gradient so that the cost can be computed as z'H*z + q'*z + c
*/
class QuadraticTrajectoryCost {
  virtual std::vector<std::vector<int>> GetStencil(std::vector<std::vector<int>>& q_index,
                                                   std::vector<std::vector<int>>& u_index, int k) = 0;

  virtual std::vector<Eigen::MatrixXd> GetM(int k, Eigen::VectorXd z_ref) = 0;

  void ConstructCostHessianAndGradient(Eigen::VectorXd z_ref);

  // Definition of the stencil blocks
  std::shared_ptr<TrajectorySQP> traj_sqp_;
  std::vector<Eigen::MatrixXd> blocks_;
};

class ConfigurationCost : public QuadraticTrajectoryCost {
  std::vector<std::vector<int>> GetStencil(std::vector<std::vector<int>>& q_index,
                                           std::vector<std::vector<int>>& u_index, int k) override;

  std::vector<Eigen::MatrixXd> GetM(int k, Eigen::VectorXd z_ref) override;
};

}  // namespace optimizer
}  // namespace idto

