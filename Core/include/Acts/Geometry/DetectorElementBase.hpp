// This file is part of the Acts project.
//
// Copyright (C) 2016-2018 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "Acts/Definitions/Algebra.hpp"
#include "Acts/Geometry/GeometryContext.hpp"
#include "Acts/Geometry/GeometryIdentifier.hpp"  

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

// Specialize std::hash for GeometryIdentifier
namespace std {
template <>
struct hash<Acts::GeometryIdentifier> {
  size_t operator()(const Acts::GeometryIdentifier& id) const {
    return std::hash<Acts::GeometryIdentifier::Value>()(id.value());
  }
};
}  // namespace std

namespace Acts {

class Surface;

class DetectorElementBase {
public:
  DetectorElementBase() = default;
  virtual ~DetectorElementBase() = default;

  virtual const Transform3& transform(const GeometryContext& gctx) const = 0;
  virtual const Surface& surface() const = 0;
  virtual Surface& surface() = 0;
  virtual double thickness() const = 0;

  void setMisalignmentParameters(GeometryIdentifier sensorId, double misalignmentX, double misalignmentY);
  std::pair<double, double> getMisalignmentParameters(GeometryIdentifier sensorId) const;

private:
  std::unordered_map<GeometryIdentifier, std::pair<double, double>> m_misalignmentParameters;
};

void DetectorElementBase::setMisalignmentParameters(GeometryIdentifier sensorId, double misalignmentX, double misalignmentY) {
  m_misalignmentParameters[sensorId] = std::make_pair(misalignmentX, misalignmentY);
}

std::pair<double, double> DetectorElementBase::getMisalignmentParameters(GeometryIdentifier sensorId) const {
  auto it = m_misalignmentParameters.find(sensorId);
  if (it != m_misalignmentParameters.end()) {
    return it->second;
  }
  return std::make_pair(0.0, 0.0);  // Default values if sensor not found
}

}  // namespace Acts
