#include "factor.h"

#include <boost/format.hpp>

#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

/* -------------------------------------------------------------------------- */
/*                                  Detection                                 */
/* -------------------------------------------------------------------------- */

Detection::Detection(Detection::BoundingBox box,
                     gtsam::Vector6 variances)
    : box(box), variances(variances) {
  auto p     = box.pose.position;
  auto q     = box.pose.orientation;
  this->pose = gtsam::Pose3(gtsam::Rot3(q.w, q.x, q.y, q.z), gtsam::Point3(p.x, p.y, p.z));
}

/* -------------------------------------------------------------------------- */

const double Detection::error(const gtsam::Pose3 x) const {
  gtsam::Vector errorVector = gtsam::traits<gtsam::Pose3>::Local(this->pose, x);
  return sqrt(this->getDiagonal()->Mahalanobis(errorVector));
}

/* -------------------------------------------------------------------------- */

std::tuple<size_t, double>
getDetectionIndexAndError(const gtsam::Pose3 &d, std::vector<Detection> detections) {
  if (detections.size() == 0) {
    throw std::runtime_error("Does not receive any detection.");
  }

  size_t idx   = 0;
  double error = detections[0].error(d);

  // Figure out the optimal detection index
  for (size_t i = 1; i < detections.size(); ++i) {
    double error_i = detections[i].error(d);
    if (error_i < error) {
      idx   = i;
      error = error_i;
    }
  }

  return std::make_tuple(idx, error);
}

/* -------------------------------------------------------------------------- */
/*                      Tightly-Coupled Detection Factor                      */
/* -------------------------------------------------------------------------- */

gtsam::Vector
TightlyCoupledDetectionFactor::evaluateError(const gtsam::Pose3 &robotPose,
                                             const gtsam::Pose3 &objectPose,
                                             const gtsam::Pose3 &measured,
                                             boost::optional<gtsam::Matrix &> H1,
                                             boost::optional<gtsam::Matrix &> H2) const {
  gtsam::Pose3 hx = gtsam::traits<gtsam::Pose3>::Between(robotPose,
                                                         objectPose,
                                                         H1,
                                                         H2);
  return gtsam::traits<gtsam::Pose3>::Local(measured, hx);
}

/* -------------------------------------------------------------------------- */

gtsam::Vector
TightlyCoupledDetectionFactor::unwhitenedError(const gtsam::Pose3 &measured,
                                               const gtsam::Values &x,
                                               boost::optional<std::vector<gtsam::Matrix> &> H) const {
  if (this->active(x)) {
    const gtsam::Pose3 &x1 = x.at<gtsam::Pose3>(robotPoseKey());
    const gtsam::Pose3 &x2 = x.at<gtsam::Pose3>(objectPoseKey());
    if (H) {
      return evaluateError(x1, x2, measured, (*H)[0], (*H)[1]);
    } else {
      return evaluateError(x1, x2, measured);
    }
  } else {
    return gtsam::Vector::Zero(this->dim());
  }
}

/* -------------------------------------------------------------------------- */

boost::shared_ptr<gtsam::GaussianFactor>
TightlyCoupledDetectionFactor::linearize(const gtsam::Values &c) const {
  // Only linearize if the factor is active
  if (!active(c))
    return boost::shared_ptr<gtsam::JacobianFactor>();

  // Determine which detection is used to generate the factor function, a.k.a.
  // the Max-Mixture model
  size_t index;
  double error;
  const auto robotPose   = c.at<gtsam::Pose3>(this->robotPoseKey());
  const auto objectPose  = c.at<gtsam::Pose3>(this->objectPoseKey());
  std::tie(index, error) = getDetectionIndexAndError(robotPose.inverse() * objectPose, this->detections);

  auto measured   = this->detections[index].getPose();
  auto noiseModel = this->noiseModels[index];

  // Call evaluate error to get Jacobians and RHS vector b
  std::vector<gtsam::Matrix> A(size());
  gtsam::Vector b = -unwhitenedError(measured, c, A);
  if (noiseModel && b.size() != noiseModel->dim())
    throw std::invalid_argument(
        boost::str(
            boost::format(
                "NoiseModelFactor: NoiseModel has dimension %1% instead of %2%.") %
            noiseModel->dim() % b.size()));

  // Whiten the corresponding system now
  if (noiseModel)
    noiseModel->WhitenSystem(A, b);

  // Fill in terms, needed to create JacobianFactor below
  std::vector<std::pair<gtsam::Key, gtsam::Matrix> > terms(size());
  for (size_t j = 0; j < size(); ++j) {
    terms[j].first = keys()[j];
    terms[j].second.swap(A[j]);
  }

  // TODO pass unwhitened + noise model to Gaussian factor
  using gtsam::noiseModel::Constrained;
  if (noiseModel && noiseModel->isConstrained())
    return gtsam::GaussianFactor::shared_ptr(
        new gtsam::JacobianFactor(terms, b,
                                  boost::static_pointer_cast<Constrained>(noiseModel)->unit()));
  else
    return gtsam::GaussianFactor::shared_ptr(new gtsam::JacobianFactor(terms, b));
}

/* -------------------------------------------------------------------------- */
/*                      Loosely-Coupled Detection Factor                      */
/* -------------------------------------------------------------------------- */

gtsam::Vector
LooselyCoupledDetectionFactor::evaluateError(const gtsam::Pose3 &robotPose,
                                             const gtsam::Pose3 &objectPose,
                                             const gtsam::Pose3 &measured,
                                             boost::optional<gtsam::Matrix &> H1) const {
  gtsam::Pose3 hx = gtsam::traits<gtsam::Pose3>::Between(robotPose,
                                                         objectPose,
                                                         boost::none,
                                                         H1);
  return gtsam::traits<gtsam::Pose3>::Local(measured, hx);
}

/* -------------------------------------------------------------------------- */

gtsam::Vector
LooselyCoupledDetectionFactor::unwhitenedError(const gtsam::Pose3 &measured,
                                               const gtsam::Values &x,
                                               boost::optional<std::vector<gtsam::Matrix> &> H) const {
  if (this->active(x)) {
    const gtsam::Pose3 &x1 = x.at<gtsam::Pose3>(this->robotPoseKey());
    const gtsam::Pose3 &x2 = x.at<gtsam::Pose3>(this->objectPoseKey());
    if (H) {
      return evaluateError(x1, x2, measured, (*H)[0]);
    } else {
      return evaluateError(x1, x2, measured);
    }
  } else {
    return gtsam::Vector::Zero(this->dim());
  }
}

/* -------------------------------------------------------------------------- */

boost::shared_ptr<gtsam::GaussianFactor>
LooselyCoupledDetectionFactor::linearize(const gtsam::Values &c) const {
  // Only linearize if the factor is active
  if (!active(c))
    return boost::shared_ptr<gtsam::JacobianFactor>();

  // Determine which detection is used to generate the factor function, a.k.a.
  // the Max-Mixture model
  size_t index;
  double error;
  const auto robotPose   = c.at<gtsam::Pose3>(this->robotPoseKey());
  const auto objectPose  = c.at<gtsam::Pose3>(this->objectPoseKey());
  std::tie(index, error) = getDetectionIndexAndError(robotPose.inverse() * objectPose, this->detections);

  auto measured   = this->detections[index].getPose();
  auto noiseModel = this->noiseModels[index];

  // Call evaluate error to get Jacobians and RHS vector b
  std::vector<gtsam::Matrix> A(size());
  gtsam::Vector b = -unwhitenedError(measured, c, A);
  if (noiseModel && b.size() != noiseModel->dim())
    throw std::invalid_argument(
        boost::str(
            boost::format(
                "NoiseModelFactor: NoiseModel has dimension %1% instead of %2%.") %
            noiseModel->dim() % b.size()));

  // Whiten the corresponding system now
  if (noiseModel)
    noiseModel->WhitenSystem(A, b);

  // Fill in terms, needed to create JacobianFactor below
  std::vector<std::pair<gtsam::Key, gtsam::Matrix> > terms(size());
  for (size_t j = 0; j < size(); ++j) {
    terms[j].first = keys()[j];
    terms[j].second.swap(A[j]);
  }

  // TODO pass unwhitened + noise model to Gaussian factor
  using gtsam::noiseModel::Constrained;
  if (noiseModel && noiseModel->isConstrained())
    return gtsam::GaussianFactor::shared_ptr(
        new gtsam::JacobianFactor(terms, b,
                                  boost::static_pointer_cast<Constrained>(noiseModel)->unit()));
  else
    return gtsam::GaussianFactor::shared_ptr(new gtsam::JacobianFactor(terms, b));
}

/* -------------------------------------------------------------------------- */
/*                             Stable Pose Factor                             */
/* -------------------------------------------------------------------------- */

gtsam::Vector
StablePoseFactor::evaluateError(const gtsam::Pose3 &previousPose,
                                const gtsam::Pose3 &velocity,
                                const gtsam::Pose3 &nextPose,
                                boost::optional<gtsam::Matrix &> H1,
                                boost::optional<gtsam::Matrix &> H2,
                                boost::optional<gtsam::Matrix &> H3) const {
  auto identity = gtsam::Pose3::identity();

  auto deltaPoseVec = gtsam::traits<gtsam::Pose3>::Local(identity, velocity) * this->deltaTime;
  auto deltaPose    = gtsam::traits<gtsam::Pose3>::Retract(identity, deltaPoseVec);
  gtsam::Pose3 hx   = nextPose.inverse() * previousPose * deltaPose;

  if (H1) *H1 = -velocity.inverse().AdjointMap();
  if (H2) *H2 = gtsam::Matrix66::Identity() * this->deltaTime;
  if (H3) *H3 = -hx.inverse().AdjointMap();

  return gtsam::traits<gtsam::Pose3>::Local(identity, hx);
}
