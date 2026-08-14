#include "halmet_digital.h"

#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/ui/config_item.h"

namespace halmet {

using namespace sensesp;

// Default RPM count scale factor, corresponds to 100 pulses per revolution.
// This is rarely, if ever correct - adjust it in the web UI
// ("Tacho <name> Multiplier") once you know the real pulses-per-revolution
// of your sender.
const float kDefaultFrequencyScale = 7.0;

FloatProducer* ConnectTachoSender(int pin, String name) {
  char config_path[80];
  char sk_path[80];
  char config_title[80];
  char config_description[80];

  snprintf(config_path, sizeof(config_path), "/Tacho %s/Pin", name.c_str());
  snprintf(config_title, sizeof(config_title), "Tacho %s Pin", name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Tacho %s Input Pin", name.c_str());
  auto tacho_input =
      new DigitalInputCounter(pin, INPUT, RISING, 500, config_path);

  ConfigItem(tacho_input)
      ->set_title(config_title)
      ->set_description(config_description);

  snprintf(config_path, sizeof(config_path), "/Tacho %s/Revolution Multiplier",
           name.c_str());
  snprintf(config_title, sizeof(config_title), "Tacho %s Multiplier",
           name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Tacho %s Multiplier", name.c_str());
  auto tacho_frequency = new Frequency(kDefaultFrequencyScale, config_path);

  ConfigItem(tacho_frequency)
      ->set_title(config_title)
      ->set_description(config_description);

  tacho_input->connect_to(tacho_frequency);

  snprintf(config_path, sizeof(config_path), "/Tacho %s/Revolutions SK Path",
           name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.revolutions",
           name.c_str());
  snprintf(config_title, sizeof(config_title), "Tacho %s Signal K Path",
           name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Tacho %s Signal K Path", name.c_str());

  auto tacho_frequency_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("Hz", "Engine " + name + " Revolutions"));

  ConfigItem(tacho_frequency_sk_output)
      ->set_title(config_title)
      ->set_description(config_description);

  tacho_frequency->connect_to(tacho_frequency_sk_output);

  return tacho_frequency;
}

BoolProducer* ConnectAlarmSender(int pin, String name) {
  char config_path[80];
  char sk_path[80];
  char config_title[80];
  char config_description[80];

  auto* alarm_input = new DigitalInputState(pin, INPUT, 100);

  // NOTE: this used to be wrapped in `#ifdef ENABLE_SIGNALK`, guarded by a
  // macro that only main.cpp defined. Since this firmware is Signal K only
  // now, the SK output is always wired up - no macro needed. (Previously,
  // once the ENABLE_SIGNALK #define was removed from main.cpp, this
  // function would have silently stopped sending anything to Signal K.)
  snprintf(config_path, sizeof(config_path), "/Alarm %s/SK Path",
           name.c_str());
  snprintf(sk_path, sizeof(sk_path), "alarm.%s", name.c_str());
  snprintf(config_title, sizeof(config_title), "Alarm %s Signal K Path",
           name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Alarm %s Signal K Path", name.c_str());

  auto alarm_sk_output = new SKOutputBool(sk_path, config_path);

  ConfigItem(alarm_sk_output)
      ->set_title(config_title)
      ->set_description(config_description);

  alarm_input->connect_to(alarm_sk_output);

  return alarm_input;
}

}  // namespace halmet
