// This file is part of the Acts project.
//
// Copyright (C) 2016-2020 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#pragma once

#include "Acts/Definitions/Algebra.hpp"
#include "Acts/Definitions/TrackParametrization.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include <vector>
#include <unordered_set>

namespace Acts {

/// Components of alignment parameters vector.
///
/// To be used to access components by named indices instead of just numbers.
/// This must be a regular `enum` and not a scoped `enum class` to allow
/// implicit conversion to an integer. The enum value are thus visible directly
/// in `namespace Acts` and are prefixed to avoid naming collisions.
enum AlignmentIndices : unsigned int {
  // Center of geometry object in global 3D cartesian coordinates
  eAlignmentCenter0 = 0u,
  eAlignmentCenter1 = eAlignmentCenter0 + 1u,
  eAlignmentCenter2 = eAlignmentCenter0 + 2u,
  // Rotation angle around global x/y/z axis of geometry object
  eAlignmentRotation0 = 3u,
  eAlignmentRotation1 = eAlignmentRotation0 + 1u,
  eAlignmentRotation2 = eAlignmentRotation0 + 2u,
  // Last uninitialized value contains the total number of components
  eAlignmentSize,
};

// Matrix and vector types related to alignment parameters.
using AlignmentVector = ActsVector<eAlignmentSize>;
using AlignmentRowVector = ActsMatrix<1, eAlignmentSize>;
using AlignmentMatrix = ActsMatrix<eAlignmentSize, eAlignmentSize>;
using AlignmentToPositionMatrix = ActsMatrix<3, eAlignmentSize>;
using AlignmentToBoundMatrix = ActsMatrix<eBoundSize, eAlignmentSize>;
using AlignmentToPathMatrix = ActsMatrix<1, eAlignmentSize>;

// Structure to hold misalignment parameters for an individual sensor
struct MisalignmentParameters {
  double centerShiftX;
  double centerShiftY;
  double centerShiftZ;
  double rotationX;
  double rotationY;
  double rotationZ;
};

// class to store misalignment parameters (multiple sensors)
class SensorMisalignments {
public:
  explicit SensorMisalignments(size_t numSensors)
      : misalignments(numSensors, MisalignmentParameters{}) {}

  MisalignmentParameters& getMisalignment(size_t sensorIndex) {
    return misalignments[sensorIndex];
  }

private:
  std::vector<MisalignmentParameters> misalignments;
};

// new class: this class will handle selective misalignment
class SuperstructureMisalignments {
public:
  void addSuperstructure(const Surface& surface) {
    selectedSuperstructures.insert(&surface);
  }

  void removeSuperstructure(const Surface& surface) {
    selectedSuperstructures.erase(&surface);
  }

  bool isSuperstructureSelected(const Surface& surface) const {
    return selectedSuperstructures.count(&surface) > 0;
  }

private:
  std::unordered_set<const Surface&> selectedSuperstructures;

};

}  // namespace Acts
