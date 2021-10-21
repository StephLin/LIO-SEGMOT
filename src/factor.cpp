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

Detection::Detection(BoundingBox box, gtsam::Vector3 sigma, double w) {
  this->box      = box;
  this->sigmaVec = sigma;
  this->sigmaMat = this->sigmaVec.matrix().asDiagonal();
  this->info     = (1 / this->sigmaVec.array()).matrix().asDiagonal();
  this->sqrtInfo = (1 / this->sigmaVec.array().sqrt()).matrix().asDiagonal();
  this->w        = w;
  this->diagonal = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << sigma(0), sigma(1), sigma(2), 1e-4, 1e-4, 1e-4).finished());

  this->mu << box.pose.position.x, box.pose.position.y, box.pose.position.z;
}

Detection::Detection(BoundingBox box, double sigma, double w) {
  this->box      = box;
  this->sigmaVec = gtsam::Vector3::Ones() * sigma;
  this->sigmaMat = this->sigmaVec.matrix().asDiagonal();
  this->info     = (1 / this->sigmaVec.array()).matrix().asDiagonal();
  this->sqrtInfo = (1 / this->sigmaVec.array().sqrt()).matrix().asDiagonal();
  this->w        = w;
  this->diagonal = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << sigma, sigma, sigma, 1e-4, 1e-4, 1e-4).finished());

  this->mu << box.pose.position.x, box.pose.position.y, box.pose.position.z;
}

/* -------------------------------------------------------------------------- */

const double Detection::error(const gtsam::Vector3 x, const double gamma) const {
  double c              = this->w * sqrt(this->info.determinant());
  double first          = -2 * (log(c / gamma));
  gtsam::Vector3 deltaX = (x - this->mu);
  double second         = deltaX.matrix().transpose() * this->info * deltaX;

  return first + second;
}

/* -------------------------------------------------------------------------- */

const gtsam::Pose3 Detection::getPose3() const {
  auto q          = this->box.pose.orientation;
  gtsam::Rot3 r   = gtsam::Rot3(q.w, q.x, q.y, q.z);  // scalar-first format
  auto p          = this->box.pose.position;
  gtsam::Point3 t = gtsam::Point3(p.x, p.y, p.z);
  return gtsam::Pose3(r, t);
}

#ifdef SAMMOT_USE_EXPLICT_NOISE_MODEL

/* -------------------------------------------------------------------------- */
/*                              Max-Mixture Model                             */
/* -------------------------------------------------------------------------- */

MaxMixtureModel::MaxMixtureModel(std::vector<Detection> detections)
    : detections(detections) {
  for (Detection detection : detections) {
    gtsam::Vector3 sigma = detection.getSigmaVec();
    this->gaussians.push_back(gtsam::noiseModel::Diagonal::Variances(sigma));
  }
}

#endif

/* -------------------------------------------------------------------------- */
/*                              Detection Factor                              */
/* -------------------------------------------------------------------------- */

DetectionFactor::DetectionFactor(std::vector<Detection> detections,
                                 gtsam::Key detectionKey,
                                 gtsam::Key robotPoseKey,
                                 DetectionFactor::Mode mode)
    : detections(detections),
      detectionKey(detectionKey),
      robotPoseKey(robotPoseKey),
      gamma(0) {
  if (detections.size() == 0) {
    throw std::runtime_error("Does not exist any detection.");
  }

  for (Detection detection : detections) {
    auto diagonal = gtsam::noiseModel::Diagonal::Variances(detection.getVarianceVec());
    this->diagonals.push_back(diagonal);

    this->zs.push_back(detection.getPose3().translation());

    // Calculate gamma value in terms of weight and information matrix.
    double &&gamma_ = detection.getW() * sqrt(detection.getInformationMatrix().determinant());
    if (gamma_ > this->gamma) {
      this->gamma = gamma_;
    }
  }
}

DetectionFactor::DetectionFactor(const This *f) {
  this->detections = f->detections;
  this->zs         = f->zs;
  this->gamma      = f->gamma;

  // TODO: Do copy-initialization instead of re-initialization.
  for (size_t idx = 0; idx < this->detections.size(); ++idx) {
    this->diagonals.push_back(gtsam::noiseModel::Diagonal::Variances(this->detections[idx].getVarianceVec()));
  }
}

/* -------------------------------------------------------------------------- */

void DetectionFactor::print(const std::string &s,
                            const gtsam::KeyFormatter &keyFormatter) const {
  std::cout << s
            << "DetectionFactor("
            << keyFormatter(this->detectionKey)
            << ", "
            << keyFormatter(this->robotPoseKey)
            << ')';
}

bool DetectionFactor::equals(const DetectionFactor::Base &f, double tol) const {
  const This *e = dynamic_cast<const This *>(&f);
  return e != NULL && Base::equals(*e, tol);
}

/* -------------------------------------------------------------------------- */

double DetectionFactor::error(const gtsam::Values &c) const {
  size_t idx;
  double error;
  std::tie(idx, error) = this->getDetectionIndexAndError(c);

  return error;
}

gtsam::GaussianFactor::shared_ptr
DetectionFactor::linearize(const Values &c) const {
  size_t detectionIndex;
  double error;

  std::tie(detectionIndex, error) = this->getDetectionIndexAndError(c);
  auto measured                   = this->detections[detectionIndex].getPose3();
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
DetectionFactor::getDetectionIndexAndError(const gtsam::Pose3 &d) const {
  size_t idx   = 0;
  double error = this->detections[0].error(d.translation(), this->gamma);

  // Figure out the optimal detection index
  for (size_t i = 1; i < this->detections.size(); ++i) {
    double error_i = this->detections[i].error(d.translation(), this->gamma);
    if (error_i > error) {
      idx   = i;
      error = error_i;
    }
  }

  return std::make_tuple(idx, error);
}

std::tuple<size_t, double>
DetectionFactor::getDetectionIndexAndError(const gtsam::Values &c) const {
  auto p = this->getDetectionValue(c);
  auto x = this->getRobotPoseValue(c);

  return this->getDetectionIndexAndError(x.inverse() * p);
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
                                boost::optional<gtsam::Matrix &> H3) {
  gtsam::Pose3 hx = nextPose.inverse() * previousPose * velocity;

  if (H1) *H1 = -velocity.inverse().AdjointMap();
  if (H2) *H2 = gtsam::Matrix66::Identity();
  if (H3) *H3 = -hx.inverse().AdjointMap();

  return gtsam::traits<gtsam::Pose3>::Local(hx, gtsam::Pose3::identity());
}
