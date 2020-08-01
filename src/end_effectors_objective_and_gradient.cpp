#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
    f = [&](const Eigen::VectorXd & A)->double
    {
      Skeleton copy = copy_skeleton_at(skeleton, A);
      Eigen::VectorXd tips = transformed_tips(copy, b);
      return (tips - xb0).dot(tips - xb0);
    };
    
    grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
    {
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(A.size());
        Skeleton copy = copy_skeleton_at(skeleton, A);
        Eigen::VectorXd tips = transformed_tips(copy, b);
        
        Eigen::MatrixXd jacobian;
        kinematics_jacobian(copy,b,jacobian);
        
        for(int i=0;i<b.size();++i)
        {
            for(int j=0;j<3;++j)
            {
                int index = 3*i+j;
                double dx = 2 * (tip_trans(index)-xb0(index));
                gradient += jacobian.row(index).transpose() * dx;
            }
        }
        return gradient;
  };
    
    proj_z = [&](Eigen::VectorXd & A)
    {
        assert(skeleton.size()*3 == A.size());
        for(int i=0;i<skeleton.size();i++)
        {
            A[3*i] = std::max(skeleton[i].xzx_min[0], std::min(skeleton[i].xzx_max[0], A[3*i]));
            A[3*i+1] = std::max(skeleton[i].xzx_min[1], std::min(skeleton[i].xzx_max[1], A[3*i+1]));
            A[3*i+2] = std::max(skeleton[i].xzx_min[2], std::min(skeleton[i].xzx_max[2], A[3*i+2]));
        }
    };
  /////////////////////////////////////////////////////////////////////////////
}
