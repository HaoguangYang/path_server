#include "path_server/bezier_interpolation.hpp"

namespace path_server {

BezierCurveInterpolator::BezierCurveInterpolator(const MatrixXd& points, bool closedPath)
    : closedPath_(closedPath), points_(points) {
  auto coeffs = getBezierCoeffs(points_);
  A_ = coeffs.first;
  B_ = coeffs.second;
  if (closedPath_) {
    nInterpSeg_ = VectorXi(points.rows());
  } else {
    nInterpSeg_ = VectorXi(points.rows() - 1);
  }
}

MatrixXd BezierCurveInterpolator::evaluate() {
  const int nSeg = closedPath_ ? points_.rows() : points_.rows() - 1;
  if (!byDistance_) {
    nTotal_ = interpNPerSeg_ * nSeg;
    for (int i = 0; i < nSeg; i++) nInterpSeg_(i) = interpNPerSeg_;
  } else {
    for (int i = 0; i < nSeg; i++) {
      nInterpSeg_(i) = std::ceil(segDists_(i) / interpDistance_);
    }
    nTotal_ = nInterpSeg_.sum();
  }
  MatrixXd result = MatrixXd(nTotal_, points_.cols());
  unsigned int ind = 0;
  for (int i = 0; i < points_.rows() - 1; i++) {
    double reciprocal = 1.0 / nInterpSeg_(i);
    double now = reciprocal;
    for (int j = 0; j < nInterpSeg_(i); j++) {
      for (int k = 0; k < points_.cols(); k++) {
        result(ind, k) =
            bezierCubicValue(points_(i, k), A_(i, k), B_(i, k), points_(i + 1, k), now);
      }
      now += reciprocal;
      ind++;
    }
  }
  if (closedPath_) {
    const int i = nSeg - 1;
    double reciprocal = 1.0 / nInterpSeg_(i);
    double now = reciprocal;
    for (int j = 0; j < nInterpSeg_(i); j++) {
      for (int k = 0; k < points_.cols(); k++) {
        result(ind, k) = bezierCubicValue(points_(i, k), A_(i, k), B_(i, k), points_(0, k), now);
      }
      now += reciprocal;
      ind++;
    }
  }
  return result;
}

MatrixXd BezierCurveInterpolator::evaluateAnother(const MatrixXd& points) {
  MatrixXd result = MatrixXd(nTotal_, points_.cols());
  if (!nTotal_) {
    return result;
  }
  // size incompatibility
  if (points.rows() != points_.rows()) {
    return result;
  }
  points_ = points;
  auto coeffs = getBezierCoeffs(points);
  A_ = coeffs.first;
  B_ = coeffs.second;
  unsigned int ind = 0;
  for (int i = 0; i < points_.rows() - 1; i++) {
    double reciprocal = 1.0 / nInterpSeg_(i);
    double now = reciprocal;
    for (int j = 0; j < nInterpSeg_(i); j++) {
      for (int k = 0; k < points_.cols(); k++) {
        result(ind, k) =
            bezierCubicValue(points_(i, k), A_(i, k), B_(i, k), points_(i + 1, k), now);
      }
      now += reciprocal;
      ind++;
    }
  }
  if (closedPath_) {
    const int i = points_.rows() - 1;
    double reciprocal = 1.0 / nInterpSeg_(i);
    double now = reciprocal;
    for (int j = 0; j < nInterpSeg_(i); j++) {
      for (int k = 0; k < points_.cols(); k++) {
        result(ind, k) = bezierCubicValue(points_(i, k), A_(i, k), B_(i, k), points_(0, k), now);
      }
      now += reciprocal;
      ind++;
    }
  }
  return result;
}

std::pair<MatrixXd, MatrixXd> BezierCurveInterpolator::getBezierCoeffs(const MatrixXd& points) {
  const int n = closedPath_ ? points.rows() : points.rows() - 1;

  MatrixXd A = MatrixXd::Zero(n, points.cols());
  for (int i = 1; i < n - 1; i++) {
    A.row(i) = 2. * (2. * points.row(i) + points.row(i + 1));
  }
  if (!closedPath_) {
    // boundary condition of first-order derivative = 0
    A.row(0) = points.row(0);
    A.row(n - 1) = points.row(n);
    // boundary condition of second-order derivative = 0
    // (https://towardsdatascience.com/b%C3%A9zier-interpolation-8033e9a262c2)
    // A.row(0) = points.row(0) + 2. * points.row(1);
    // A.row(n - 1) = 8. * points.row(n - 1) + points.row(n);
  } else {
    A.row(0) = 2. * (2. * points.row(0) + points.row(1));
    A.row(n - 1) = 2. * (2. * points.row(n - 1) + points.row(0));
  }
  for (int i = 0; i < points.cols(); i++) {
    VectorXd a = VectorXd::Ones(n);
    VectorXd b = 4. * VectorXd::Ones(n);
    VectorXd c = VectorXd::Ones(n);
    if (!closedPath_) {
      // boundary condition of first-order derivative = 0
      a(0) = 0.;
      a(n - 1) = 0.;
      c(0) = 0.;
      c(n - 1) = 0.;
      b(0) = 1.;
      b(n - 1) = 1.;
      // boundary condition of second-order derivative = 0
      // (https://towardsdatascience.com/b%C3%A9zier-interpolation-8033e9a262c2)
      // a(0) = 0.;
      // c(n-1) = 0.;
      // b(0) = 2.; b(n-1) = 7.; a(n-1) = 2.;
    }
    VectorXd sol = A.col(i);
    solveTriDiagSystem(a, b, c, sol);
    A.col(i) = sol;
  }
  MatrixXd B = MatrixXd::Zero(n, points.cols());
  for (int i = 0; i < n - 1; i++) {
    B.row(i) = 2. * points.row(i + 1) - A.row(i + 1);
  }
  if (!closedPath_) {
    // boundary condition of first-order derivative = 0
    B.row(n - 1) = 2. * points.row(n) - A.row(n - 1);
    // boundary condition of second-order derivative = 0
    // (https://towardsdatascience.com/b%C3%A9zier-interpolation-8033e9a262c2)
    // B.row(n - 1) = (A.row(n - 1) + points.row(n)) * 0.5;
  } else {
    B.row(n - 1) = 2. * points.row(0) - A.row(0);
  }
  return std::make_pair(A, B);
}

void BezierCurveInterpolator::solveTriDiagSystem(VectorXd& a, VectorXd& b, VectorXd& c,
                                                 VectorXd& y) {
  // size checking -- a : lower diagonal; b : main diagonal; c : upper diagonal
  // a(0) -- upper left corner for cyclic boundary condition
  // c(n-1) -- lower right corner for cyclic boundary condition
  if (y.size() != b.size()) {
    return;
  }
  if (b.size() != a.size()) {
    return;
  }
  if (a.size() != c.size()) {
    return;
  }
  // solve it
  if (a(0) != 0. || c.tail(1).value() != 0.) {
    // apply cyclic boundary condition
    // assert (b(0)!=0);
    // Using Sherman–Morrison formula + Thomas Algorithm to solve.
    VectorXd u = VectorXd::Zero(b.size());
    VectorXd v = VectorXd::Zero(b.size());
    // double gamma = -b(0): -- this causes severe oscillation.
    double gamma = -b(0);
    u(0) = gamma;
    u.tail(1) << c.tail(1).value();
    v(0) = 1.0;
    v.tail(1) << a(0) / gamma;
    b(0) -= gamma;
    b.tail(1) << b.tail(1).value() - c.tail(1).value() * a(0) / gamma;
    a(0) = 0.;
    c.tail(1).setZero();
    // Need two solutions. Therefore make a copy.
    VectorXd a1 = a;
    VectorXd b1 = b;
    VectorXd c1 = c;
    solveTriDiagSystem(a, b, c, y);
    solveTriDiagSystem(a1, b1, c1, u);
    y -= v.dot(y) / (1.0 + v.dot(u)) * u;
    return;
  }
  // Thomas algorithm for pure tri-diagonal matrix.
  for (size_t i = 1; i < static_cast<size_t>(b.size()); i++) {
    a(i) /= b(i - 1);
    b(i) -= a(i) * c(i - 1);
    y(i) -= a(i) * y(i - 1);
  }
  y(b.size() - 1) /= b(b.size() - 1);
  for (size_t i = b.size() - 1; i != 0; i--) {
    y(i - 1) = (y(i - 1) - c(i - 1) * y(i)) / b(i - 1);
  }
}

}  // namespace path_server
