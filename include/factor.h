// Copyright 2021 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.
#include <stdexcept>
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
  gtsam::Vector6 variances;

 public:
  Detection(BoundingBox box, gtsam::Vector6 variances);

  const BoundingBox getBoundingBox() const { return this->box; }
  const gtsam::Pose3 getPose() const { return this->pose; };
  gtsam::noiseModel::Diagonal::shared_ptr getDiagonal() const { return gtsam::noiseModel::Diagonal::Variances(this->variances); }

  const double error(const gtsam::Pose3 x) const;
};

std::tuple<size_t, double>
getDetectionIndexAndError(const gtsam::Pose3 &d,
                          std::vector<Detection> detections);

class TightlyCoupledDetectionFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3,
                                                                      gtsam::Pose3> {
 private:
  using This = TightlyCoupledDetectionFactor;
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;

 protected:
  std::vector<Detection> detections;
  std::vector<gtsam::noiseModel::Diagonal::shared_ptr> noiseModels;

 public:
  /** Constructor */
  TightlyCoupledDetectionFactor(gtsam::Key robotPoseKey,
                                gtsam::Key objectPoseKey,
                                std::vector<Detection> detections)
      : Base(boost::shared_ptr<gtsam::noiseModel::Diagonal>(),
             robotPoseKey,
             objectPoseKey),
        detections(detections) {
    for (const auto &detection : detections) {
      this->noiseModels.push_back(detection.getDiagonal());
    }
  }

  /** print */
  virtual void
  print(const std::string &s,
        const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "TightlyCoupledDetectionFactor("
              << keyFormatter(this->robotPoseKey()) << ", "
              << keyFormatter(this->objectPoseKey()) << ")\n";
  }

  /** equals */
  virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != NULL && Base::equals(*e, tol);
  }

  gtsam::Key robotPoseKey() const { return key1(); }
  gtsam::Key objectPoseKey() const { return key2(); }

  virtual gtsam::Vector
  evaluateError(const gtsam::Pose3 &robotPose,
                const gtsam::Pose3 &objectPose,
                const gtsam::Pose3 &measured,
                boost::optional<gtsam::Matrix &> H1 = boost::none,
                boost::optional<gtsam::Matrix &> H2 = boost::none) const;

  virtual gtsam::Vector
  evaluateError(const gtsam::Pose3 &robotPose,
                const gtsam::Pose3 &objectPose,
                boost::optional<gtsam::Matrix &> H1 = boost::none,
                boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
    throw std::runtime_error("This evaluateError function is revoked.");
  }

  virtual gtsam::Vector unwhitenedError(const gtsam::Pose3 &measured,
                                        const gtsam::Values &x,
                                        boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const;

  virtual boost::shared_ptr<gtsam::GaussianFactor>
  linearize(const gtsam::Values &c) const override;
};

class LooselyCoupledDetectionFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  using This = LooselyCoupledDetectionFactor;
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

 protected:
  std::vector<Detection> detections;
  std::vector<gtsam::noiseModel::Diagonal::shared_ptr> noiseModels;

  gtsam::Key robotPoseKey_;

 public:
  /** Constructor */
  LooselyCoupledDetectionFactor(gtsam::Key robotPoseKey,
                                gtsam::Key objectPoseKey,
                                std::vector<Detection> detections)
      : Base(boost::shared_ptr<gtsam::noiseModel::Diagonal>(), objectPoseKey),
        robotPoseKey_(robotPoseKey),
        detections(detections) {
    for (const auto &detection : detections) {
      this->noiseModels.push_back(detection.getDiagonal());
    }
  }

  /** print */
  virtual void
  print(const std::string &s,
        const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "LooselyCoupledDetectionFactor("
              << keyFormatter(this->robotPoseKey()) << ","
              << keyFormatter(this->objectPoseKey()) << ")\n";
  }

  /** equals */
  virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != NULL && Base::equals(*e, tol);
  }

  gtsam::Key robotPoseKey() const { return this->robotPoseKey_; }
  gtsam::Key objectPoseKey() const { return this->key(); }

  virtual gtsam::Vector
  evaluateError(const gtsam::Pose3 &robotPose,
                const gtsam::Pose3 &objectPose,
                const gtsam::Pose3 &measured,
                boost::optional<gtsam::Matrix &> H1 = boost::none) const;

  virtual gtsam::Vector
  evaluateError(const gtsam::Pose3 &x, boost::optional<gtsam::Matrix &> H = boost::none) const override {
    throw std::runtime_error("This evaluateError function is revoked.");
  }

  virtual gtsam::Vector unwhitenedError(const gtsam::Pose3 &measured,
                                        const gtsam::Values &x,
                                        boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const;

  virtual boost::shared_ptr<gtsam::GaussianFactor>
  linearize(const gtsam::Values &c) const override;
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

 protected:
  double deltaTime;

 public:
  /** Constructor */
  StablePoseFactor(gtsam::Key previousPoseKey,
                   gtsam::Key velocityKey,
                   gtsam::Key nextPoseKey,
                   double deltaTime,
                   const gtsam::SharedNoiseModel &model = nullptr)
      : Base(model, previousPoseKey, velocityKey, nextPoseKey), deltaTime(deltaTime) {
  }

  /** print */
  virtual void
  print(const std::string &s,
        const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "StablePoseFactor("
              << keyFormatter(this->key1()) << ","
              << keyFormatter(this->key2()) << ","
              << keyFormatter(this->key3()) << ")"
              << "  dt = " << this->deltaTime << '\n';
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