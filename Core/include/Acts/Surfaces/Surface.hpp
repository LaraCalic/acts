// This file is part of the Acts project.
//
// Copyright (C) 2016-2020 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <cstdint>
#include <functional>
#include <iosfwd>
#include <utility>
#include <unordered_map>

class Surface;

namespace Actss {

/// Identifier for geometry nodes within the geometry hierarchy.
///
/// An identifier can be split into the following components. They define
/// a hierarchy of objects starting from the high-level volumes:
///
/// - Volume
/// - Boundary surfaces (for a volume)
/// - Layers (confined within a volume)
/// - Approach surfaces (for a layer)
/// - Sensitive surfaces (confined to a layer, also called modules)
///
class NewGeometryIdentifier {
 public:
  using Value = uint64_t;

  /// Construct from an already encoded value.
  constexpr NewGeometryIdentifier(Value encoded)
      : m_value(encoded), m_misalignmentX(0.0), m_misalignmentY(0.0) {}
  /// Construct default GeometryIdentifier with all values set to zero.
  NewGeometryIdentifier()
      : m_value(0), m_misalignmentX(0.0), m_misalignmentY(0.0) {}
  NewGeometryIdentifier(NewGeometryIdentifier&&) = default;
  NewGeometryIdentifier(const NewGeometryIdentifier&) = default;
  ~NewGeometryIdentifier() = default;
  NewGeometryIdentifier& operator=(NewGeometryIdentifier&&) = default;
  NewGeometryIdentifier& operator=(const NewGeometryIdentifier&) = default;

  /// Return the encoded value.
  constexpr Value value() const { return m_value; }

  /// Return the volume identifier.
  constexpr Value volume() const { return getBits(kVolumeMask); }
  /// Return the boundary identifier.
  constexpr Value boundary() const { return getBits(kBoundaryMask); }
  /// Return the layer identifier.
  constexpr Value layer() const { return getBits(kLayerMask); }
  /// Return the approach identifier.
  constexpr Value approach() const { return getBits(kApproachMask); }
  /// Return the sensitive identifier.
  constexpr Value sensitive() const { return getBits(kSensitiveMask); }
  /// Return the extra identifier
  /// Usage can be experiment-specific, like tagging which kind of detector a
  /// surface object corresponds to, or which subsystem it belongs to
  constexpr Value extra() const { return getBits(kExtraMask); }

  /// Set the volume identifier.
  constexpr NewGeometryIdentifier& setVolume(Value volume) {
    return setBits(kVolumeMask, volume);
  }
  /// Set the boundary identifier.
  constexpr NewGeometryIdentifier& setBoundary(Value boundary) {
    return setBits(kBoundaryMask, boundary);
  }
  /// Set the layer identifier.
  constexpr NewGeometryIdentifier& setLayer(Value layer) {
    return setBits(kLayerMask, layer);
  }
  /// Set the approach identifier.
  constexpr NewGeometryIdentifier& setApproach(Value approach) {
    return setBits(kApproachMask, approach);
  }
  /// Set the sensitive identifier.
  constexpr NewGeometryIdentifier& setSensitive(Value sensitive) {
    return setBits(kSensitiveMask, sensitive);
  }
  /// Set the extra identifier
  constexpr NewGeometryIdentifier& setExtra(Value extra) {
    return setBits(kExtraMask, extra);
  }

  /// Return the misalignment in x- direction
  double misalignmentX() const { return m_misalignmentX; }
  /// Return the misalignment in y-direction (for the surface)
  double misalignmentY() const { return m_misalignmentY; }
  /// Setting the misalignment in x-direction 
  void setMisalignmentX(double misalignmentX) {
    m_misalignmentX = misalignmentX;
  }
  /// Setting the misalignment in y - direction (for the surface)
  void setMisalignmentY(double misalignmentY) {
    m_misalignmentY = misalignmentY;
  }

 private:
  // clang-format off
  static constexpr Value kVolumeMask    = 0xff00000000000000; // (2^8)-1 = 255 volumes
  static constexpr Value kBoundaryMask  = 0x00ff000000000000; // (2^8)-1 = 255 boundaries
  static constexpr Value kLayerMask     = 0x0000fff000000000; // (2^12)-1 = 4095 layers
  static constexpr Value kApproachMask  = 0x0000000ff0000000; // (2^8)-1 = 255 approach surfaces
  static constexpr Value kSensitiveMask = 0x000000000fffff00; // (2^20)-1 = 1048575 sensitive surfaces
  static constexpr Value kExtraMask     = 0x00000000000000ff; // (2^8)-1 = 255 extra values
  // clang-format on

  Value m_value = 0;
  double m_misalignmentX = 0.0;
  double m_misalignmentY = 0.0;

  /// Extract the bit shift necessary to access the masked values.
  static constexpr int extractShift(Value mask) {
    // use compiler builtin to extract the number of trailing bits from the
    // mask. the builtin should be available on all supported compilers.
    // need unsigned long long version (...ll) to ensure uint64_t compatibility.
    // WARNING undefined behaviour for mask == 0 which we should not have.
    return __builtin_ctzll(mask);
  }
  /// Extract the masked bits from the encoded value.
  constexpr Value getBits(Value mask) const {
    return (m_value & mask) >> extractShift(mask);
  }
  /// Set the masked bits to id in the encoded value.
  constexpr NewGeometryIdentifier& setBits(Value mask, Value id) {
    m_value = (m_value & ~mask) | ((id << extractShift(mask)) & mask);
    // return *this here so we need to write fewer lines in the set... methods
    return *this;
  }

  friend constexpr bool operator==(NewGeometryIdentifier lhs,
                                   NewGeometryIdentifier rhs) {
    return lhs.m_value == rhs.m_value;
  }
  friend constexpr bool operator!=(NewGeometryIdentifier lhs,
                                   NewGeometryIdentifier rhs) {
    return lhs.m_value != rhs.m_value;
  }
  friend constexpr bool operator<(NewGeometryIdentifier lhs,
                                  NewGeometryIdentifier rhs) {
    return lhs.m_value < rhs.m_value;
  }
  friend constexpr bool operator<=(NewGeometryIdentifier lhs,
                                   NewGeometryIdentifier rhs) {
    return lhs.m_value <= rhs.m_value;
  }
  friend constexpr bool operator>(NewGeometryIdentifier lhs,
                                  NewGeometryIdentifier rhs) {
    return lhs.m_value > rhs.m_value;
  }
  friend constexpr bool operator>=(NewGeometryIdentifier lhs,
                                   NewGeometryIdentifier rhs) {
    return lhs.m_value >= rhs.m_value;
  }
};

}  // namespace Actss
