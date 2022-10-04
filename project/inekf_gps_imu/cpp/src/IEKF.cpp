#include <IEKF.hpp>
#include <utils.hpp>

#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <iostream>

using namespace std::chrono;
using namespace Eigen;
using std::cos;
using std::pow;
using std::sin;

namespace iekf
{
// Shortand for matrices and vectors
// consistent with Eigen
using Matrix5d = IEKF::Matrix5d;
using Matrix15d = IEKF::Matrix15d;
using Matrix9d = Eigen::Matrix<double, 9, 9>;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using Timestamp = IEKF::Timestamp;

void IEKF::resetFilter(const Timestamp& time, const Vector3d& origin_lla)
{
    origin_ = origin_lla;
    origin_set_ = true;
    time_ = time;
    time_last_predict_ = time;
}

void IEKF::prediction(
    const Vector3d& acc, const Vector3d& gyro, duration<double> dt_dur)
{
 
}

void IEKF::correction(const Vector3d& gps_lla)
{

}

void IEKF::addImu(
    const Timestamp& timestamp, const Vector3d& acc, const Vector3d& gyro)
{
   
}

void IEKF::addGps(const Timestamp& timestamp, const Vector3d& gps)
{
   
}

std::tuple<Matrix5d&, Matrix15d&, Timestamp&> IEKF::getState()
{
    return std::tie(mu_, Sigma_, time_);
}

}  // namespace iekf