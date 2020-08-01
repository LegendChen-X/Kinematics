#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
    J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);
    Eigen::VectorXd tips = transformed_tips(skeleton, b);
    double h = 1e-7;
    for(int i=0;i<skeleton.size();++i)
    {
        for(int j=0;j<3;++j)
        {
            Skeleton buff = skeleton;
            buff[i].xzx[j] += h;
            Eigen::VectorXd h_tips = transformed_tips(buff, b);
            J.col(3*i+j) = (h_tips-tips)/h;
        }
    }
  /////////////////////////////////////////////////////////////////////////////
}
