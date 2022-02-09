// Copyright 2022 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.
#pragma once

#include <gtsam/nonlinear/ISAM2.h>

#include "factor.h"

class MaxMixtureISAM2 : public gtsam::ISAM2 {
 public:
  explicit MaxMixtureISAM2(const gtsam::ISAM2Params& params) : gtsam::ISAM2(params){};

  MaxMixtureISAM2() : gtsam::ISAM2(){};

  virtual gtsam::ISAM2Result update(
      const gtsam::NonlinearFactorGraph& newFactors                            = gtsam::NonlinearFactorGraph(),
      const gtsam::Values& newTheta                                            = gtsam::Values(),
      const gtsam::FactorIndices& removeFactorIndices                          = gtsam::FactorIndices(),
      const boost::optional<gtsam::FastMap<gtsam::Key, int> >& constrainedKeys = boost::none,
      const boost::optional<gtsam::FastList<gtsam::Key> >& noRelinKeys         = boost::none,
      const boost::optional<gtsam::FastList<gtsam::Key> >& extraReelimKeys     = boost::none,
      bool force_relinearize                                                   = false) override;

  /**
   * Add variable, update, and relinearize if need; almost everything is same as
   * the original ISAM2 solver, except for the relinearization condition for
   * max-mixture models.
   */
  virtual gtsam::ISAM2Result
  update(const gtsam::NonlinearFactorGraph& newFactors,
         const gtsam::Values& newTheta,
         const gtsam::ISAM2UpdateParams& updateParams) override;
};