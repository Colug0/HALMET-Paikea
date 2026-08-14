#ifndef __SRC_HALMET_DIGITAL_H__
#define __SRC_HALMET_DIGITAL_H__

#include "sensesp/sensors/sensor.h"

namespace halmet {

/**
 * @brief Connect a digital input pin as an engine tachometer and wire its
 * frequency output directly to a Signal K path
 * (propulsion.<name>.revolutions).
 */
sensesp::FloatProducer* ConnectTachoSender(int pin, String name);

/**
 * @brief Connect a digital input pin as a boolean alarm input and wire it
 * directly to a Signal K path (alarm.<name>).
 */
sensesp::BoolProducer* ConnectAlarmSender(int pin, String name);

}  // namespace halmet

#endif
