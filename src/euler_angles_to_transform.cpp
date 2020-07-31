#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
    Eigen::Affine3d A;
    Eigen::Affine3d B;
    Eigen::Affine3d C;
    
    double th0 = xzx[0]*M_PI/180;
    double th1 = xzx[1]*M_PI/180;
    double th2 = xzx[2]*M_PI/180;
    
    A.matrix() <<
    1,0,0,0,
    0,cos(th0),-sin(th0),0,
    0,sin(th0),cos(th0),0,
    0,0,0,1;
    
    B.matrix() <<
    cos(th1),-sin(th1),0,0,
    sin(th1),cos(th1),0,0,
    0,0,1,0,
    0,0,0,1;
    
    C.matrix() <<
    1,0,0,0,
    0,cos(th2),-sin(th2),0,
    0,sin(th2),cos(th2),0,
    0,0,0,1;
    
    return C*B*A;
  /////////////////////////////////////////////////////////////////////////////
}
