#pragma once

#include "Acts/Utilities/EnumBitwiseOperators.hpp"

#include <limits>
#include <type_traits>
#include <vector>

//acts/Alignment/include/ActsAlignment/Kernel/AlignmentMask.hpp

namespace ActsAlignment {

/// Collection of bit masks to enable steering which alignment degree of freedom
/// should be float. should be initialized, and which should be left invalid.
/// These mask values can be combined using binary operators, so
/// (AlignmentMask::Center0 | AlignmentMask::Center1) will instruct
/// alignment for geometry object center x and y
/// The enum is used as a strong type wrapper around the bits to prevent
/// autoconversion from integer
enum struct AlignmentMask : uint8_t {
  None = 0,
  Center0 = 1 << 0,
  Center1 = 1 << 1,
  Center2 = 1 << 2,
  Rotation0 = 1 << 3,
  Rotation1 = 1 << 4,
  Rotation2 = 1 << 5,

  All = std::numeric_limits<uint8_t>::max(),  // should be all ones
};

ACTS_DEFINE_ENUM_BITWISE_OPERATORS(AlignmentMask)


struct MisalignmentParameters {
  double centerShiftX;
  double centerShiftY;
  double centerShiftZ;
  double rotationX;
  double rotationY;
  double rotationZ;
};

class SensorMisalignments {
public:
  explicit SensorMisalignments(size_t numSensors)
      : misalignments(numSensors, MisalignmentParameters{}) {}

  // misalignment parameters for a specific sensor
  MisalignmentParameters& getMisalignment(size_t sensorIndex) {
    return misalignments[sensorIndex];
  }

private:
  std::vector<MisalignmentParameters> misalignments;
};

}  // namespace ActsAlignment
