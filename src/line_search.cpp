#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
    double res = max_step;
    Eigen::VectorXd move = z - res * dz;
    proj_z(move);
    while(f(move) > f(z))
    {
        res = res / 2;
        move = z - res * dz;
        proj_z(move);
    }
    return res;
  /////////////////////////////////////////////////////////////////////////////
}
