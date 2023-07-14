// This file is part of the Acts project.
//
// Copyright (C) 2019 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "ActsExamples/ContextualDetector/InternalAlignmentDecorator.hpp"
#include "Acts/Definitions/Algebra.hpp"
#include "Acts/Geometry/GeometryContext.hpp"
#include "ActsExamples/ContextualDetector/InternallyAlignedDetectorElement.hpp"
#include "ActsExamples/Framework/AlgorithmContext.hpp"
#include "ActsExamples/Framework/RandomNumbers.hpp"
#include <algorithm> // For std::find

// acts/Examples/Detectors/ContextualDetector/src/InternalAlignmentDecorator.cpp

ActsExamples::ProcessCode
ActsExamples::Contextual::InternalAlignmentDecorator::decorate(
    AlgorithmContext& context) {
  // We need to lock the Decorator
  std::lock_guard<std::mutex> alignmentLock(m_alignmentMutex);

  // In which iov batch are we?
  unsigned int iov = context.eventNumber / m_cfg.iovSize;

  ACTS_VERBOSE("IOV handling in thread " << std::this_thread::get_id() << ".");
  ACTS_VERBOSE("IOV resolved to " << iov << " - from event "
                                  << context.eventNumber << ".");

  m_eventsSeen++;

  context.geoContext = InternallyAlignedDetectorElement::ContextType{iov};

  if (m_cfg.randomNumberSvc != nullptr) {
    if (auto it = m_activeIovs.find(iov); it != m_activeIovs.end()) {
      // Iov is already present, update last accessed
      it->second.lastAccessed = m_eventsSeen;
    } else {
      // Iov is not present yet, create it

      m_activeIovs.emplace(iov, IovStatus{m_eventsSeen});

      ACTS_VERBOSE("New IOV " << iov << " detected at event "
                              << context.eventNumber
                              << ", emulate new alignment.");

      //algorithm local random number generator
      RandomEngine rng = m_cfg.randomNumberSvc->spawnGenerator(context);

      for (auto& lstore : m_cfg.detectorStore) {
        for (auto& ldet : lstore) {
          // if the current superstructure in the list of selected ones?
          if (std::find(m_cfg.selectedSuperstructures.begin(), m_cfg.selectedSuperstructures.end(), ldet->superstructureId) != m_cfg.selectedSuperstructures.end()) {
            // get the nominal transform
            Acts::Transform3 tForm =
                ldet->nominalTransform(context.geoContext);  // copy
            // create a new transform
            applyTransform(tForm, m_cfg, rng, iov);

            // for each, individual sensor, generate the correleted misalignment 
            auto misalignments = generateCorrelatedMisalignments(
                m_cfg.sigmaInPlane, m_cfg.sigmaOutPlane, m_cfg.sigmaInRot,
                m_cfg.sigmaOutRot, rng);

            // applying misalignments to the transform
            applyMisalignment(tForm, std::get<0>(misalignments),
                              std::get<1>(misalignments),
                              std::get<2>(misalignments),
                              std::get<3>(misalignments));

            // puting back transformed alignment back into the store
            ldet->addAlignedTransform(tForm, iov);
          }
        }
      }
    }
  }

  // Garbage collection
  if (m_cfg.doGarbageCollection) {
    for (auto it = m_activeIovs.begin(); it != m_activeIovs.end();) {
      unsigned int this_iov = it->first;
      auto& status = it->second;
      if (m_eventsSeen - status.lastAccessed > m_cfg.flushSize) {
        ACTS_DEBUG("IOV " << this_iov << " has not been accessed in the last "
                          << m_cfg.flushSize << " events, clearing");
        it = m_activeIovs.erase(it);
        for (auto& lstore : m_cfg.detectorStore) {
          for (auto& ldet : lstore) {
            ldet->clearAlignedTransform(this_iov);
          }
        }
      } else {
        it++;
      }
    }
  }

  return ProcessCode::SUCCESS;
}
