#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function

Eigen::Affine3d get_transformation(const Skeleton & skeleton, int index)
{
    if(skeleton[index].parent_index == -1) return Eigen::Affine3d::Identity();
    
    Eigen::Affine3d parent_transformation = get_transformation(skeleton, skeleton[index].parent_index);
    
    Eigen::Affine3d rest_T = skeleton[index].rest_T;
    Eigen::Affine3d rotation = euler_angles_to_transform(skeleton[index].xzx);
    
    return parent_transformation * skeleton[index].rest_T * rotation * skeleton[index].rest_T.inverse();
}


void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
    T.resize(skeleton.size(),Eigen::Affine3d::Identity());
    for(int i=0;i<skeleton.size();++i) T[i] = get_transformation(skeleton,i);
  /////////////////////////////////////////////////////////////////////////////
}
