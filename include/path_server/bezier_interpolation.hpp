#ifndef _PATH_SERVER__BEZIER_INTERPOLATION_HPP_
#define _PATH_SERVER__BEZIER_INTERPOLATION_HPP_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace Eigen;
// using Eigen::internal::BandMatrix;

namespace path_server {
class BezierCurveInterpolator {
 public:
  BezierCurveInterpolator(const MatrixXd& points, bool closedPath = false);

  ~BezierCurveInterpolator() = default;

  void setInterpDistance(double val) {
    byDistance_ = true;
    interpDistance_ = val;
    segDists_ = VectorXd(nInterpSeg_.size());
    for (int i = 0; i < points_.rows() - 1; i++) {
      VectorXd diff = points_.row(i + 1) - points_.row(i);
      segDists_(i) = std::sqrt(diff.dot(diff));
    }
    if (closedPath_) {
      VectorXd diff = points_.row(points_.rows() - 1) - points_.row(0);
      segDists_(points_.rows() - 1) = std::sqrt(diff.dot(diff));
    }
  };

  void setInterpNPerSegment(unsigned int val) {
    byDistance_ = false;
    interpNPerSeg_ = val;
  };

  MatrixXd evaluate();

  MatrixXd evaluateAnother(const MatrixXd& points);

  const VectorXd& getSegDists() { return segDists_; }

  const VectorXi& getSegNInterp() { return nInterpSeg_; }

 private:
  std::pair<MatrixXd, MatrixXd> getBezierCoeffs(const MatrixXd& points);

  /**
   * @brief Using Thomas method to implement a tri-diagonal Ax=y solver in O(n). In-place solver,
   * result stored in y.
   *
   * @param a Lower diagonal vector
   * @param b Main diagonal vector
   * @param c Upper diagonal vector
   * @param y Input: right hand side of the equation. Output: in-place solution of X.
   */
  void solveTriDiagSystem(VectorXd& a, VectorXd& b, VectorXd& c, VectorXd& y);

  double bezierCubicValue(const double& a, const double& b, const double& c, const double& d,
                          const double& t) {
    const double oneMinusT = 1. - t;
    const double tSquared = t * t;
    const double oneMinusTSq = oneMinusT * oneMinusT;
    const double tCubed = tSquared * t;
    const double oneMinusTCu = oneMinusTSq * oneMinusT;
    return a * oneMinusTCu + 3. * b * t * oneMinusTSq + 3. * c * tSquared * oneMinusT + d * tCubed;
  };

  bool closedPath_;
  MatrixXd points_;
  bool byDistance_ = false;
  double interpDistance_ = 1.0;
  unsigned int interpNPerSeg_ = 100;
  MatrixXd A_, B_;
  VectorXd segDists_;
  unsigned int nTotal_ = 0;
  VectorXi nInterpSeg_;
};

}  // namespace path_server

#endif  // _PATH_SERVER__BEZIER_INTERPOLATION_HPP_
