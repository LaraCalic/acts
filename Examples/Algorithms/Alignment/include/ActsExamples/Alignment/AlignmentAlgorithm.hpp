// This file is part of the Acts project.
//
// Copyright (C) 2019 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
//Examples/Algorithms/Alignment/include/ActsExamples/Alignment/AlignmentAlgorithm.hpp

// This file is part of the Acts project.
//
// Copyright (C) 2019 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "Acts/TrackFitting/KalmanFitter.hpp"
#include "ActsAlignment/Kernel/Alignment.hpp"
#include "ActsExamples/EventData/IndexSourceLink.hpp"
#include "ActsExamples/EventData/Measurement.hpp"
#include "ActsExamples/EventData/ProtoTrack.hpp"
#include "ActsExamples/EventData/Track.hpp"
#include "ActsExamples/Framework/DataHandle.hpp"
#include "ActsExamples/Framework/IAlgorithm.hpp"
#include "ActsExamples/MagneticField/MagneticField.hpp"

#include <functional>
#include <memory>
#include <vector>

namespace ActsExamples {

class AlignmentAlgorithm final : public IAlgorithm {
 public:
  using AlignmentResult = Acts::Result<ActsAlignment::AlignmentResult>;
  using AlignmentParameters =
      std::unordered_map<Acts::DetectorElementBase*, Acts::Transform3>;

  using SuperstructureAlignmentParameters =
      std::unordered_map<const Acts::Surface*, Acts::Transform3>;

  // Alignment function that takes sets of input measurements, initial
  // trackstate and alignment options and returns some alignment-specific
  // result.
  using TrackFitterOptions =
      Acts::KalmanFitterOptions<Acts::VectorMultiTrajectory>;

  class AlignmentFunction {
   public:
    virtual ~AlignmentFunction() = default;
    virtual AlignmentResult operator()(
        const std::vector<std::vector<IndexSourceLink>>&,
        const TrackParametersContainer&,
        const ActsAlignment::AlignmentOptions<TrackFitterOptions>&,
        const SuperstructureAlignmentParameters&) const = 0;
  };

  static std::shared_ptr<AlignmentFunction> makeAlignmentFunction(
      std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
      std::shared_ptr<const Acts::MagneticFieldProvider> magneticField);

  struct Config {
    std::string inputMeasurements;
    std::string inputSourceLinks;
    std::string inputProtoTracks;
    std::string inputInitialTrackParameters;
    std::string outputAlignmentParameters;
    std::shared_ptr<AlignmentFunction> align;
    ActsAlignment::AlignedTransformUpdater alignedTransformUpdater;
    std::vector<Acts::DetectorElementBase*> alignedDetElements;
    std::map<unsigned int, std::bitset<6>> iterationState;
    double chi2ONdfCutOff = 0.10;
    std::pair<size_t, double> deltaChi2ONdfCutOff = {10, 0.00001};
    size_t maxNumIterations = 100;
    int maxNumTracks = -1;
    // Add a new member for superstructure alignment parameters
    SuperstructureAlignmentParameters superstructureAlignmentParameters;
  };

  AlignmentAlgorithm(Config cfg, Acts::Logging::Level lvl);

  ActsExamples::ProcessCode execute(
      const ActsExamples::AlgorithmContext& ctx) const override;

 private:
  Config m_cfg;

  ReadDataHandle<MeasurementContainer> m_inputMeasurements{this,
                                                           "InputMeasurements"};
  ReadDataHandle<IndexSourceLinkContainer> m_inputSourceLinks{
      this, "InputSourceLinks"};
  ReadDataHandle<TrackParametersContainer> m_inputInitialTrackParameters{
      this, "InputInitialTrackParameters"};
  ReadDataHandle<ProtoTrackContainer> m_inputProtoTracks{this,
                                                         "InputProtoTracks"};
  WriteDataHandle<AlignmentParameters> m_outputAlignmentParameters{
      this, "OutputAlignmentParameters"};
};

}  // namespace ActsExamples
