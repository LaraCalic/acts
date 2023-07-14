#include "Acts/Surfaces/Surface.hpp"
#include "Acts/EventData/detail/TransformationBoundToFree.hpp"
#include "Acts/EventData/detail/CorrectedTransformationFreeToBound.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include "Acts/Definitions/Algebra.hpp"
#include "Acts/Definitions/Common.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include "Acts/Utilities/UnitVectors.hpp"

#include <algorithm>

Acts::FreeVector Acts::detail::transformBoundToFreeParameters(
    const Acts::Surface& surface, const GeometryContext& geoCtx,
    const Acts::BoundVector& boundParams) {
  // convert angles to global unit direction vector
  Vector3 direction = makeDirectionUnitFromPhiTheta(boundParams[eBoundPhi],
                                                    boundParams[eBoundTheta]);

  // convert local position to global position vector
  Vector2 local(boundParams[eBoundLoc0], boundParams[eBoundLoc1]);
  Vector3 position = surface.localToGlobal(geoCtx, local, direction);

  // construct full free-vector. time and q/p stay as-is.
  FreeVector freeParams = FreeVector::Zero();
  freeParams[eFreePos0] = position[ePos0];
  freeParams[eFreePos1] = position[ePos1];
  freeParams[eFreePos2] = position[ePos2];
  freeParams[eFreeTime] = boundParams[eBoundTime];
  freeParams[eFreeDir0] = direction[eMom0];
  freeParams[eFreeDir1] = direction[eMom1];
  freeParams[eFreeDir2] = direction[eMom2];
  freeParams[eFreeQOverP] = boundParams[eBoundQOverP];
  return freeParams;
}
