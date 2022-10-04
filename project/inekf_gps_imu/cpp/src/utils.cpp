#include <cmath>
#include <utils.hpp>

using namespace Eigen;

using Matrix5d = Matrix<double, 5, 5>;
using Vector9d = Matrix<double, 9, 1>;

/*****************************************************************************/
// clang-format off
Matrix3d skew(const Vector3d& u) {
    return (Matrix3d() << 
            0, -u(2),  u(1), 
         u(2),     0, -u(0), 
        -u(1),  u(0),     0).finished();
}
// clang-format on

/*****************************************************************************/
Matrix3d gamma0(const Vector3d& phi)
{

}

/*****************************************************************************/
Matrix3d gamma1(const Vector3d& phi)
{

}

/*****************************************************************************/
Matrix3d gamma2(const Vector3d& phi)
{

}

/*****************************************************************************/
Matrix5d makeTwist(const Vector9d& u)
{

}

Eigen::Vector3d lla_to_ecef(Eigen::Vector3d lla)
{

}

Eigen::Vector3d lla_to_enu(Eigen::Vector3d lla, Eigen::Vector3d origin_lla)
{

}