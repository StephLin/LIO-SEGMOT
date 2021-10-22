#include "factor.h"

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
                     gtsam::noiseModel::Diagonal::shared_ptr diagonal)
    : box(box), diagonal(diagonal) {
  auto p     = box.pose.position;
  auto q     = box.pose.orientation;
  this->pose = gtsam::Pose3(gtsam::Rot3(q.w, q.x, q.y, q.z), gtsam::Point3(p.x, p.y, p.z));
}

/* -------------------------------------------------------------------------- */

const double Detection::error(const gtsam::Pose3 x) const {
  gtsam::Vector errorVector = gtsam::traits<gtsam::Pose3>::Local(this->pose, x);
  return sqrt(this->diagonal->Mahalanobis(errorVector));
}

/* -------------------------------------------------------------------------- */

std::tuple<size_t, double>
getDetectionIndexAndError(const gtsam::Pose3 &d, std::vector<Detection> detections) {
  size_t idx   = 0;
  double error = detections[0].error(d);

  // Figure out the optimal detection index
  for (size_t i = 1; i < detections.size(); ++i) {
    double error_i = detections[i].error(d);
    if (error_i > error) {
      idx   = i;
      error = error_i;
    }
  }

  return std::make_tuple(idx, error);
}

/* -------------------------------------------------------------------------- */
/*                              Detection Factor                              */
/* -------------------------------------------------------------------------- */

DetectionFactor::DetectionFactor(std::vector<Detection> detections,
                                 gtsam::Key robotPoseKey,
                                 gtsam::Key detectionKey,
                                 DetectionFactor::Mode mode)
    : detections(detections),
      robotPoseKey(robotPoseKey),
      detectionKey(detectionKey) {
  if (detections.size() == 0) {
    throw std::runtime_error("Does not exist any detection.");
  }
}

DetectionFactor::DetectionFactor(const This *f) {
  this->detections   = f->detections;
  this->robotPoseKey = f->robotPoseKey;
  this->detectionKey = f->detectionKey;
}

/* -------------------------------------------------------------------------- */

void DetectionFactor::print(const std::string &s,
                            const gtsam::KeyFormatter &keyFormatter) const {
  std::cout << s
            << "DetectionFactor("
            << keyFormatter(this->detectionKey) << ", "
            << keyFormatter(this->robotPoseKey) << ')';
}

bool DetectionFactor::equals(const DetectionFactor::Base &f, double tol) const {
  const This *e = dynamic_cast<const This *>(&f);
  return e != NULL && Base::equals(*e, tol);
}

/* -------------------------------------------------------------------------- */

double DetectionFactor::error(const gtsam::Values &c) const {
  size_t idx;
  double error;
  std::tie(idx, error) = this->getDetectionIndexAndErrorBasedOnStates(c);

  return error;
}

gtsam::GaussianFactor::shared_ptr
DetectionFactor::linearize(const Values &c) const {
  size_t detectionIndex;
  double error;

  std::tie(detectionIndex, error) = this->getDetectionIndexAndErrorBasedOnStates(c);
  auto measured                   = this->detections[detectionIndex].getPose();
  auto diagonal                   = this->detections[detectionIndex].getDiagonal();

  if (this->mode == Mode::LOOSELY_COUPLED) {
    auto factor = gtsam::BetweenFactor<gtsam::Pose3>(this->robotPoseKey,
                                                     this->detectionKey,
                                                     measured,
                                                     diagonal);
    return factor.linearize(c);
  } else if (this->mode == Mode::TIGHTLY_COUPLED) {
    measured    = this->getRobotPoseValue(c) * measured;
    auto factor = gtsam::PriorFactor<gtsam::Pose3>(this->detectionKey,
                                                   measured,
                                                   diagonal);
    return factor.linearize(c);
  }
}

gtsam::NonlinearFactor::shared_ptr DetectionFactor::clone() const {
  return boost::make_shared<This>(this);
}

/* -------------------------------------------------------------------------- */

std::tuple<size_t, double>
DetectionFactor::getDetectionIndexAndErrorBasedOnStates(const gtsam::Values &c) const {
  auto p = this->getDetectionValue(c);
  auto x = this->getRobotPoseValue(c);

  return getDetectionIndexAndError(x.inverse() * p, this->detections);
}

/* -------------------------------------------------------------------------- */

const gtsam::Pose3
DetectionFactor::getDetectionValue(const gtsam::Values &c) const {
  const gtsam::Pose3 *p = dynamic_cast<const gtsam::Pose3 *>(&(c.at(this->detectionKey)));
  if (p == NULL) {
    throw std::runtime_error(gtsam::DefaultKeyFormatter(this->detectionKey) + " cannot cast to Pose3.");
  }
  return *p;
}

const gtsam::Pose3
DetectionFactor::getRobotPoseValue(const gtsam::Values &c) const {
  const gtsam::Pose3 *x = dynamic_cast<const gtsam::Pose3 *>(&(c.at(this->robotPoseKey)));
  if (x == NULL) {
    throw std::runtime_error(gtsam::DefaultKeyFormatter(this->robotPoseKey) + " cannot cast to Pose3.");
  }
  return *x;
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
  gtsam::Pose3 hx = nextPose.inverse() * previousPose * velocity;

  if (H1) *H1 = -velocity.inverse().AdjointMap();
  if (H2) *H2 = gtsam::Matrix66::Identity();
  if (H3) *H3 = -hx.inverse().AdjointMap();

  return gtsam::traits<gtsam::Pose3>::Local(hx, gtsam::Pose3::identity());
}
