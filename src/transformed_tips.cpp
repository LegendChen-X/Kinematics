#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
    std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> T;
    Eigen::VectorXd tips = Eigen::VectorXd::Zero(3*b.size());
    forward_kinematics(skeleton, T);
    
    for(int i=0;i<b.size();++i)
    {
       
        Bone this_bone = skeleton[b[i]];
        Eigen::Vector4d tip(this_bone.length, 0, 0, 1);
        
        Eigen::Vector4d tip_trans = T[b[i]] * this_bone.rest_T * tip;
        
        tips[i*3] = tip_trans[0];
        tips[i*3 + 1] = tip_trans[1];
        tips[i*3 + 2] = tip_trans[2];
    }
    
    return tips;
  /////////////////////////////////////////////////////////////////////////////
}
