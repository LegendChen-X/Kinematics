#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
    //https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
    //http://paulbourke.net/miscellaneous/interpolation/
    /////////////////////////////////////////////////////////////////////////////
    // Replace with your code
    if (keyframes.size() == 0) return Eigen::Vector3d(0,0,0);
    
    int i =0;
    
    for(;i<keyframes.size() && keyframes[i].first <= fmod(t,keyframes.back().first);++i);
    
    Eigen::Vector3d p0, p1, p2, p3;
    double t0, t1, t2, t3;
    if(i == 1 || i == keyframes.size()-1)
    {
        p0 = keyframes[i-1].second;
        p1 = keyframes[i].second;
        t0 = keyframes[i-1].first;
        t1 = keyframes[i].first;
        
        double _0_ = (t-t0) / (t1-t0);
        double _2_ = (1 - cos(_0_ * M_PI));
        return p1 * _2_ + p0 * (1 - _2_);
    }
    
    p0 = keyframes[i-2].second;
    p1 = keyframes[i-1].second;
    p2 = keyframes[i].second;
    p3 = keyframes[i+1].second;

    t0 = keyframes[i-2].first;
    t1 = keyframes[i-1].first;
    t2 = keyframes[i].first;
    t3 = keyframes[i+1].first;
    
    Eigen::Vector3d a1 = (t1-t) / (t1-t0) * p0 + (t-t0) / (t1-t0) * p1;
    Eigen::Vector3d a2 = (t2-t) / (t2-t1) * p1 + (t-t1) / (t2-t1) * p2;
    Eigen::Vector3d a3 = (t3-t) / (t3-t2) * p2 + (t-t2) / (t3-t2) * p3;

    Eigen::Vector3d b1 = (t2-t) / (t2-t0) * a1 + (t-t0) / (t2-t0) * a2;
    Eigen::Vector3d b2 = (t3-t) / (t3-t1) * a2 + (t-t1) / (t3-t1) * a3;

    Eigen::Vector3d c = (t2-t) / (t2-t1) * b1 + (t-t1) / (t2-t1) * b2;
    
    return c;
    /////////////////////////////////////////////////////////////////////////////
}
