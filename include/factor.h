// Copyright 2021 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.
#include <tuple>
#include <vector>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <Eigen/Dense>

class MaxMixturePoint3 : public gtsam::Point3 {};

class Detection {
 public:
  using BoundingBox = jsk_recognition_msgs::BoundingBox;

 protected:
  BoundingBox box;
  gtsam::Pose3 pose;
  gtsam::noiseModel::Diagonal::shared_ptr diagonal;

 public:
  Detection(BoundingBox box, gtsam::noiseModel::Diagonal::shared_ptr diagonal);

  const BoundingBox getBoundingBox() const { return this->box; }
  const gtsam::Pose3 getPose() const { return this->pose; };
  const gtsam::noiseModel::Diagonal::shared_ptr getDiagonal() const { return this->diagonal; }

  const double error(const gtsam::Pose3 x) const;
};

std::tuple<size_t, double>
getDetectionIndexAndError(const gtsam::Pose3 &d,
                          std::vector<Detection> detections);

class DetectionFactor : public gtsam::NonlinearFactor {
 private:
  using Key            = gtsam::Key;
  using Values         = gtsam::Values;
  using GaussianFactor = gtsam::GaussianFactor;

 public:
  enum class MODE {
    TIGHTLY_COUPLED,
    LOOSELY_COUPLED
  };

 protected:
  using This = DetectionFactor;
  using Base = gtsam::NonlinearFactor;

  Key robotPoseKey;
  Key objectKey;

  std::vector<Detection> detections;

  MODE mode;

 public:
  DetectionFactor(std::vector<Detection> detections,
                  Key robotPoseKey,
                  Key objectKey,
                  MODE mode = MODE::LOOSELY_COUPLED);

  DetectionFactor(const This *f);

  virtual ~DetectionFactor() {}

  ///@name Testable
  ///@{

  virtual void print(const std::string &s                    = "",
                     const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override;

  virtual bool equals(const Base &f, double tol = 1e-9) const override;

  ///@}
  ///@name Standard Interface
  ///@{

  virtual double error(const Values &c) const override;

  virtual size_t dim() const override { return 3; };

  virtual GaussianFactor::shared_ptr linearize(const Values &c) const override;

  virtual Base::shared_ptr clone() const override;

  ///@}
  ///@name Max-Mixture
  ///@{

  virtual std::tuple<size_t, double>
  getDetectionIndexAndErrorBasedOnStates(const gtsam::Values &c) const;

  ///@}
  ///@name Utilities
  ///@{

  virtual const gtsam::Pose3 getDetectionValue(const gtsam::Values &c) const;

  virtual const gtsam::Pose3 getRobotPoseValue(const gtsam::Values &c) const;

  ///@}
};

class ConstantVelocityFactor : public gtsam::BetweenFactor<gtsam::Pose3> {
 private:
  using This = ConstantVelocityFactor;
  using Base = gtsam::BetweenFactor<gtsam::Pose3>;

 public:
  ConstantVelocityFactor(gtsam::Key key1,
                         gtsam::Key key2,
                         const gtsam::SharedNoiseModel &model = nullptr)
      : Base(key1, key2, gtsam::Pose3::identity(), model) {
  }

  virtual ~ConstantVelocityFactor() {}

  ///@name Testable
  ///@{

  /** print */
  virtual void
  print(const std::string &s,
        const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "ConstantVelocityFactor("
              << keyFormatter(this->key1()) << ","
              << keyFormatter(this->key2()) << ")\n";
    this->noiseModel_->print("  noise model: ");
  }

  /** equals */
  virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != NULL && Base::equals(*e, tol);
  }
};

class StablePoseFactor : public gtsam::NoiseModelFactor3<gtsam::Pose3,
                                                         gtsam::Pose3,
                                                         gtsam::Pose3> {
 private:
  using This = StablePoseFactor;
  using Base = gtsam::NoiseModelFactor3<gtsam::Pose3,
                                        gtsam::Pose3,
                                        gtsam::Pose3>;

 public:
  /** Constructor */
  StablePoseFactor(gtsam::Key previousPoseKey,
                   gtsam::Key velocityKey,
                   gtsam::Key nextPoseKey,
                   const gtsam::SharedNoiseModel &model = nullptr)
      : Base(model, previousPoseKey, velocityKey, nextPoseKey) {
  }

  /** print */
  virtual void
  print(const std::string &s,
        const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "StablePoseFactor("
              << keyFormatter(this->key1()) << ","
              << keyFormatter(this->key2()) << ","
              << keyFormatter(this->key3()) << ")\n";
    this->noiseModel_->print("  noise model: ");
  }

  /** equals */
  virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != NULL && Base::equals(*e, tol);
  }

  inline gtsam::Key previousPoseKey() const { return key1(); }
  inline gtsam::Key velocityKey() const { return key2(); }
  inline gtsam::Key nextPoseKey() const { return key3(); }

  virtual gtsam::Vector
  evaluateError(const gtsam::Pose3 &previousPose,
                const gtsam::Pose3 &velocity,
                const gtsam::Pose3 &nextPose,
                boost::optional<gtsam::Matrix &> H1 = boost::none,
                boost::optional<gtsam::Matrix &> H2 = boost::none,
                boost::optional<gtsam::Matrix &> H3 = boost::none) const override;
};