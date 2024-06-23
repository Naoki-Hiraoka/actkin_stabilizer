#ifndef AutoStabilizer_MathUtil_H
#define AutoStabilizer_MathUtil_H

#include <Eigen/Eigen>

namespace actkin_stabilizer{
  namespace mathutil {
    // axisとlocalaxisはノルムが1, mは回転行列でなければならない.
    // axisとlocalaxisがピッタリ180反対向きの場合、回転方向が定まらないので不安定
    Eigen::Matrix3d orientCoordToAxis(const Eigen::Matrix3d& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis = Eigen::Vector3d::UnitZ());

    Eigen::Transform<double, 3, Eigen::AffineCompact> orientCoordToAxis(const Eigen::Transform<double, 3, Eigen::AffineCompact>& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis = Eigen::Vector3d::UnitZ());

    // coordsとweightsのサイズは同じでなければならない
    Eigen::Vector3d calcMidPos(const std::vector<Eigen::Vector3d>& coords, const std::vector<double>& weights);

    // coordsとweightsのサイズは同じでなければならない
    Eigen::Matrix3d calcMidRot(const std::vector<Eigen::Matrix3d>& coords, const std::vector<double>& weights);

    // coordsとweightsのサイズは同じでなければならない
    Eigen::Transform<double, 3, Eigen::AffineCompact> calcMidCoords(const std::vector<Eigen::Transform<double, 3, Eigen::AffineCompact>>& coords, const std::vector<double>& weights);

    template<typename T>
    inline T clamp(const T& value, const T& limit_value) {
      return std::max(-limit_value, std::min(limit_value, value));
    }
    template<typename T>
    inline T clamp(const T& value, const T& llimit_value, const T& ulimit_value) {
      return std::max(llimit_value, std::min(ulimit_value, value));
    }
    template<typename Derived>
    inline typename Derived::PlainObject clampMatrix(const Eigen::MatrixBase<Derived>& value, const Eigen::MatrixBase<Derived>& limit_value) {
      return value.array().max(-limit_value.array()).min(limit_value.array());
    }
    template<typename Derived>
    inline typename Derived::PlainObject clampMatrix(const Eigen::MatrixBase<Derived>& value, const Eigen::MatrixBase<Derived>& llimit_value, const Eigen::MatrixBase<Derived>& ulimit_value) {
      return value.array().max(llimit_value.array()).min(ulimit_value.array());
    }

    // hullとcontoursは同一オブジェクトでもよい. 内部で単精度浮動小数点数を使う
    void calcConvexHull(const std::vector<Eigen::Vector2d>& contours, std::vector<Eigen::Vector2d>& hull);

    bool isInsideHull(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& contours);

    std::vector<Eigen::Vector2d> calcIntersectConvexHull(const std::vector<Eigen::Vector2d>& P, const std::vector<Eigen::Vector2d>& Q);

  };

};

#endif
