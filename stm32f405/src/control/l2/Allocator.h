#pragma once

#include "control/l2/Types.h"

namespace robofish::control::l2 {

// Root of the allocation tree: tail / propulsion / pectoral. It is the sole
// caller of those nodes; leaves neither call siblings nor access hardware.
// All output angles remain nominal; electrical installation polarity is L3's job.
Result allocate(const Request &request, const Recipe &recipe,
                const Geometry &geometry, const ActuatorFrame &lastCommitted,
                State &state, uint32_t nowMs, uint32_t elapsedMs);

}  // namespace robofish::control::l2
