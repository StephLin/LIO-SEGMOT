#include "solver.h"
#include <gtsam/nonlinear/ISAM2-impl.h>

gtsam::KeySet gatherMaxMixtureRelinearizationKeys(const gtsam::NonlinearFactorGraph nonlinearFactors,
                                                  const gtsam::Values theta,
                                                  const gtsam::VectorValues delta,
                                                  gtsam::KeySet* markedKeys) {
  gtsam::KeySet relinKeys;

  LooselyCoupledDetectionFactor* lcdf;
  TightlyCoupledDetectionFactor* tcdf;
  for (const auto& factor : nonlinearFactors) {
    lcdf = dynamic_cast<LooselyCoupledDetectionFactor*>(factor.get());
    tcdf = dynamic_cast<TightlyCoupledDetectionFactor*>(factor.get());

    // SKip if the factor is not based on a max-mixture model. Currently we
    // only support max-mixture models for ``LooselyCoupledDetectionFactor``
    // and ``TightlyCoupledDetectionFactor``
    if (lcdf == nullptr && tcdf == nullptr) continue;

    int index;
    int cachedIndex;
    double error;
    const std::vector<Detection>* detections;
    gtsam::Key robotPoseKey;
    gtsam::Key objectPoseKey;
    gtsam::Pose3 robotPose;
    gtsam::Pose3 objectPose;

    if (lcdf != nullptr) {
      detections    = &lcdf->getDetections();
      robotPoseKey  = lcdf->robotPoseKey();
      objectPoseKey = lcdf->objectPoseKey();
      cachedIndex   = lcdf->getCachedDetectionIndex();
    } else {  // tcdf != nullptr
      detections    = &tcdf->getDetections();
      robotPoseKey  = tcdf->robotPoseKey();
      objectPoseKey = tcdf->objectPoseKey();
      cachedIndex   = tcdf->getCachedDetectionIndex();
    }

    robotPose  = gtsam::traits<gtsam::Pose3>::Retract(theta.at<gtsam::Pose3>(robotPoseKey),
                                                      delta.at(robotPoseKey));
    objectPose = gtsam::traits<gtsam::Pose3>::Retract(theta.at<gtsam::Pose3>(objectPoseKey),
                                                      delta.at(objectPoseKey));

    std::tie(index, error) = getDetectionIndexAndError(robotPose.inverse() * objectPose, *detections);

    if (index != cachedIndex) {
      relinKeys.insert(robotPoseKey);
      relinKeys.insert(objectPoseKey);
      markedKeys->insert(robotPoseKey);
      markedKeys->insert(objectPoseKey);
    }
  }

  return relinKeys;
}

gtsam::ISAM2Result
MaxMixtureISAM2::update(
    const gtsam::NonlinearFactorGraph& newFactors,
    const gtsam::Values& newTheta,
    const gtsam::FactorIndices& removeFactorIndices,
    const boost::optional<gtsam::FastMap<gtsam::Key, int> >& constrainedKeys,
    const boost::optional<gtsam::FastList<gtsam::Key> >& noRelinKeys,
    const boost::optional<gtsam::FastList<gtsam::Key> >& extraReelimKeys,
    bool force_relinearize) {
  gtsam::ISAM2UpdateParams params;
  params.constrainedKeys     = constrainedKeys;
  params.extraReelimKeys     = extraReelimKeys;
  params.force_relinearize   = force_relinearize;
  params.noRelinKeys         = noRelinKeys;
  params.removeFactorIndices = removeFactorIndices;

  return update(newFactors, newTheta, params);
}

gtsam::ISAM2Result
MaxMixtureISAM2::update(const gtsam::NonlinearFactorGraph& newFactors,
                        const gtsam::Values& newTheta,
                        const gtsam::ISAM2UpdateParams& updateParams) {
  gttic(ISAM2_update);
  this->update_count_ += 1;
  gtsam::UpdateImpl::LogStartingUpdate(newFactors, *this);
  gtsam::ISAM2Result result(params_.enableDetailedResults);
  gtsam::UpdateImpl update(params_, updateParams);

  // Update delta if we need it to check relinearization later
  if (update.relinarizationNeeded(update_count_))
    updateDelta(updateParams.forceFullSolve);

  // 1. Add any new factors \Factors:=\Factors\cup\Factors'.
  update.pushBackFactors(newFactors, &nonlinearFactors_, &linearFactors_,
                         &variableIndex_, &result.newFactorsIndices,
                         &result.keysWithRemovedFactors);
  update.computeUnusedKeys(newFactors, variableIndex_,
                           result.keysWithRemovedFactors, &result.unusedKeys);

  // 2. Initialize any new variables \Theta_{new} and add
  // \Theta:=\Theta\cup\Theta_{new}.
  addVariables(newTheta, result.details());
  if (params_.evaluateNonlinearError)
    update.error(nonlinearFactors_, calculateEstimate(), &result.errorBefore);

  // 3. Mark linear update
  update.gatherInvolvedKeys(newFactors, nonlinearFactors_,
                            result.keysWithRemovedFactors, &result.markedKeys);
  update.updateKeys(result.markedKeys, &result);

  gtsam::KeySet relinKeys;
  result.variablesRelinearized = 0;
  if (update.relinarizationNeeded(update_count_)) {
    // 4. Mark keys in \Delta above threshold \beta:
    relinKeys = update.gatherRelinearizeKeys(roots_, delta_, fixedVariables_,
                                             &result.markedKeys);
    // also mark keys whose max-mixture model is changed
    relinKeys.merge(gatherMaxMixtureRelinearizationKeys(nonlinearFactors_,
                                                        theta_,
                                                        delta_,
                                                        &result.markedKeys));

    update.recordRelinearizeDetail(relinKeys, result.details());
    if (!relinKeys.empty()) {
      // 5. Mark cliques that involve marked variables \Theta_{J} and ancestors.
      update.findFluid(roots_, relinKeys, &result.markedKeys, result.details());
      // 6. Update linearization point for marked variables:
      // \Theta_{J}:=\Theta_{J}+\Delta_{J}.
      gtsam::UpdateImpl::ExpmapMasked(delta_, relinKeys, &theta_);
    }
    result.variablesRelinearized = result.markedKeys.size();
  }

  // 7. Linearize new factors
  update.linearizeNewFactors(newFactors, theta_, nonlinearFactors_.size(),
                             result.newFactorsIndices, &linearFactors_);
  update.augmentVariableIndex(newFactors, result.newFactorsIndices,
                              &variableIndex_);

  // 8. Redo top of Bayes tree and update data structures
  recalculate(updateParams, relinKeys, &result);
  if (!result.unusedKeys.empty()) removeVariables(result.unusedKeys);
  result.cliques = this->nodes().size();

  if (params_.evaluateNonlinearError)
    update.error(nonlinearFactors_, calculateEstimate(), &result.errorAfter);
  return result;
}
