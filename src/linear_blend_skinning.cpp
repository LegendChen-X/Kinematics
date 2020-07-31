#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
    U.resize(V.rows(),3);
    for(int i=0;i<V.rows();++i)
    {
        Eigen::Vector4d sum(0,0,0,0);
        Eigen::Vector4d vertex(V(i,0), V(i,1), V(i,2), 1);
        for(int j=0;j<skeleton.size();++j)
        {
            if(skeleton[j].weight_index != -1)
                sum += T[j] * vertex * W(i, skeleton[j].weight_index);
        }
        U(i, 0) = sum(0) / sum(3);
        U(i, 1) = sum(1) / sum(3);
        U(i, 2) = sum(2) / sum(3);
    }
  /////////////////////////////////////////////////////////////////////////////
}
