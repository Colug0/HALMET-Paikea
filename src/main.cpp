// HALMET firmware - Signal K only, no NMEA 2000.
//
// Refactored from the original HALMET-example-firmware based main.cpp:
//  - Removed all NMEA 2000 / CAN (TWAI) code. Running the TWAI driver
//    alongside WiFi on the ESP32 is a known source of long-term instability
//    (interrupt/timing contention) and is completely unnecessary here since
//    all values only need to reach Signal K.
//  - Added enable_wifi_watchdog() - SensESP's built-in watchdog that
//    reboots/reconnects if the WiFi/SK connection is lost for too long.
//    This is very likely the fix for the "loses connection after hours or
//    days" symptom.
//  - Added system info sensors (uptime, free heap, WiFi RSSI) so you can see
//    in Signal K whether free heap is slowly draining (a leak) in the days
//    before a disconnect - very useful for further diagnosis if the
//    watchdog alone doesn't fully solve it.
//  - Disabled WiFi modem sleep, which can otherwise contribute to dropped
//    long-lived connections.
//  - Sensor init failures now log an error and skip that sensor instead of
//    halting the whole firmware forever with while(1) delay(1).
//  - Removed dead/commented-out code (old NMEA2000 senders, disabled GNSS,
//    duplicate RPM approaches).

#include <Adafruit_ADS1X15.h>

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "M5UnitENV.h"

using namespace sensesp;
using namespace halmet;
using namespace sensesp::onewire;

/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.

TwoWire* i2c;

DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);
SHT4X sht4;
BMP280 bmp;

bool bmp_ok = false;
bool sht4_ok = false;

// Set the ADS1115 GAIN to adjust the analog input voltage range.
// On HALMET, this refers to the voltage range of the ADS1115 input
// AFTER the 33.3/3.3 voltage divider.

// GAIN_TWOTHIRDS: 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// GAIN_ONE:       1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// GAIN_TWO:       2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// GAIN_FOUR:      4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// GAIN_EIGHT:     8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// GAIN_SIXTEEN:   16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

const adsGain_t kADS1115Gain = GAIN_ONE;

/////////////////////////////////////////////////////////////////////
// Functions
// Callback functions for i2c sensors. Guarded so a missing/failed sensor
// doesn't report bogus values as if it were working.
float read_temp_callback() {
  if (!bmp_ok) return NAN;
  return bmp.readTemperature() + 273.15;  // convert to Kelvin
}
float read_press_callback() {
  if (!bmp_ok) return NAN;
  return bmp.readPressure();
}
float read_humid_callback() {
  if (!sht4_ok) return NAN;
  sht4.update();
  return sht4.humidity / 100.0f;
}

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
  // ESP_LOG_VERBOSE floods the serial port and costs CPU time in normal
  // operation. Use ESP_LOG_INFO day-to-day and switch to DEBUG/VERBOSE only
  // while actively troubleshooting.
  SetupLogging(ESP_LOG_INFO);

  // Define how often SensESP should read the sensor(s), in milliseconds.
  uint read_delay = 1000;             // OneWire sensors
  unsigned int read_interval = 2000;  // Environment sensors (temp/humid/baro)

  Serial.begin(115200);

  /////////////////////////////////////////////////////////////////////
  wifi_auth_mode_t auth = WIFI_AUTH_WPA_PSK;
  WiFi.setMinSecurity(auth);

  // Construct the global SensESPApp() object.
  SensESPAppBuilder builder;

  // enable_wifi_watchdog() returns a `const SensESPAppBuilder*`, unlike
  // every other builder method - chaining anything after it (even
  // get_app()) fails to compile. Call it as its own statement on the
  // still-non-const `builder` object instead, discarding the return value.
  // Reboots/reconnects automatically if the WiFi or Signal K connection is
  // lost for too long - this directly targets the "loses connection after
  // hours or days" symptom.
  builder.enable_wifi_watchdog();

  sensesp_app = (&builder)
                    ->set_hostname("halmet")
                    ->set_wifi_client("Paikea", "2001BestesBootderWelt!")
                    ->set_sk_server("192.168.88.111", 3000)
                    ->enable_ota("!HalmetSecretWiFiOTApass")
                    // Uptime, free heap and WiFi RSSI show up as Signal K
                    // paths. Watch free heap over a few days - if it trends
                    // downward, there's a leak somewhere in the sensor
                    // pipeline and the watchdog above is only a workaround.
                    ->enable_system_info_sensors()
                    ->get_app();

  // Disable WiFi modem sleep. Power-save mode is a common contributor to
  // long-lived connections quietly dying on the ESP32.
  WiFi.setSleep(false);

  // Initialize the I2C bus at 400kHz - the BMP280/SHT4x below re-run
  // Wire.begin() internally regardless (that's just how the M5Unit-ENV
  // library is written, nothing to fix on our end), so starting at the
  // same frequency here avoids an unnecessary 100kHz -> 400kHz switch
  // mid-setup. The "Bus already started in Master Mode" log warning you'll
  // still see from those calls is harmless.
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin, 400000U);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();

  ads1115->setGain(kADS1115Gain);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  ESP_LOGI(__FILENAME__, "ADS1115 initialized: %d", ads_initialized);

  //////////////////////////////////////////
  // OneWire temperature sensors

  auto alternator_temp =
      new OneWireTemperature(dts, read_delay, "/alternatorTemperature/oneWire");

  auto refrigerator_temp = new OneWireTemperature(
      dts, read_delay, "/refrigeratorTemperature/oneWire");

  auto engine_temp =
      new OneWireTemperature(dts, read_delay, "/engineTemperature/oneWire");

  // Engine temperature

  ConfigItem(engine_temp)
      ->set_title("Engine Temperature")
      ->set_description("Temperature of the engine")
      ->set_sort_order(100);

  auto engine_temp_calibration =
      new Linear(1.0, 0.0, "/engineTemperature/linear");

  ConfigItem(engine_temp_calibration)
      ->set_title("Engine Temperature Calibration")
      ->set_description("Calibration for the engine temperature sensor")
      ->set_sort_order(200);

  auto engine_temp_sk_output = new SKOutputFloat(
      "propulsion.0.temperature", "/engineTemperature/skPath",
      new SKMetadata("K", "Engine Temperature"));

  ConfigItem(engine_temp_sk_output)
      ->set_title("Engine Temperature Signal K Path")
      ->set_description("Signal K path for the engine temperature")
      ->set_sort_order(300);

  engine_temp->connect_to(engine_temp_calibration)
      ->connect_to(engine_temp_sk_output);

  // Refrigerator temperature

  ConfigItem(refrigerator_temp)
      ->set_title("Refrigerator Temperature")
      ->set_description("Temperature of the refrigerator")
      ->set_sort_order(100);

  auto refrigerator_temp_calibration =
      new Linear(1.0, 0.0, "/refrigeratorTemperature/linear");

  ConfigItem(refrigerator_temp_calibration)
      ->set_title("Refrigerator Temperature Calibration")
      ->set_description("Calibration for the refrigerator temperature sensor")
      ->set_sort_order(200);

  auto refrigerator_temp_sk_output =
      new SKOutputFloat("environment.inside.refrigerator.temperature",
                         "/refrigeratorTemperature/skPath",
                         new SKMetadata("K", "Refrigerator Temperature"));

  ConfigItem(refrigerator_temp_sk_output)
      ->set_title("Refrigerator Temperature Signal K Path")
      ->set_description("Signal K path for the refrigerator temperature")
      ->set_sort_order(300);

  refrigerator_temp->connect_to(refrigerator_temp_calibration)
      ->connect_to(refrigerator_temp_sk_output);

  // Alternator temperature

  ConfigItem(alternator_temp)
      ->set_title("Alternator Temperature")
      ->set_description("Temperature of the alternator")
      ->set_sort_order(100);

  auto alternator_temp_calibration =
      new Linear(1.0, 0.0, "/alternatorTemperature/linear");

  ConfigItem(alternator_temp_calibration)
      ->set_title("Alternator Temperature Calibration")
      ->set_description("Calibration for the alternator temperature sensor")
      ->set_sort_order(200);

  auto alternator_temp_sk_output =
      new SKOutputFloat("electrical.alternator.temperature",
                         "/alternatorTemperature/skPath",
                         new SKMetadata("K", "Alternator Temperature"));

  ConfigItem(alternator_temp_sk_output)
      ->set_title("Alternator Temperature Signal K Path")
      ->set_description("Signal K path for the alternator temperature")
      ->set_sort_order(300);

  alternator_temp->connect_to(alternator_temp_calibration)
      ->connect_to(alternator_temp_sk_output);

  //////////////////////////////////////////
  // I2C environment sensors (BMP280 + SHT40)
  //
  // NOTE: previously a failed begin() here halted the firmware forever
  // with while(1) delay(1). That means a single cold-boot I2C hiccup would
  // brick the device until manually power-cycled - and it would never
  // reconnect to WiFi/Signal K either. Now we just log the failure and
  // skip creating the associated sensors; everything else keeps running.

  bmp_ok = bmp.begin(i2c, BMP280_I2C_ADDR, kSDAPin, kSCLPin, 400000U);
  if (!bmp_ok) {
    ESP_LOGE(__FILENAME__, "Couldn't find BMP280 - skipping inside temperature/pressure");
  } else {
    bmp.setSampling(BMP280::MODE_NORMAL,      // Operating mode
                     BMP280::SAMPLING_X2,      // Temp. oversampling
                     BMP280::SAMPLING_X16,     // Pressure oversampling
                     BMP280::FILTER_X16,       // Filtering
                     BMP280::STANDBY_MS_500);  // Standby time
  }

  sht4_ok = sht4.begin(i2c, SHT40_I2C_ADDR_44, kSDAPin, kSCLPin, 400000U);
  if (!sht4_ok) {
    ESP_LOGE(__FILENAME__, "Couldn't find SHT4x - skipping inside humidity");
  } else {
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
  }

  if (bmp_ok) {
    // Temperature from BMP280
    auto* inside_temp =
        new RepeatSensor<float>(read_interval, read_temp_callback);

    auto inside_temp_sk_output = new SKOutputFloat(
        "environment.inside.temperature", "/insideTemperature/skPath",
        new SKMetadata("K", "Inside Temperature"));

    ConfigItem(inside_temp_sk_output)
        ->set_title("Inside Temperature Signal K Path")
        ->set_description("Signal K path for the temperature inside the boat")
        ->set_sort_order(300);

    inside_temp->connect_to(inside_temp_sk_output);

    // Pressure from BMP280
    auto* inside_pressure =
        new RepeatSensor<float>(read_interval, read_press_callback);

    auto inside_pressure_sk_output = new SKOutputFloat(
        "environment.inside.pressure", "/insidePressure/skPath",
        new SKMetadata("Pa", "Inside Pressure"));
    ConfigItem(inside_pressure_sk_output)
        ->set_title("Inside Pressure Signal K Path")
        ->set_description("Signal K path for the pressure inside the boat")
        ->set_sort_order(400);
    inside_pressure->connect_to(inside_pressure_sk_output);
  }

  if (sht4_ok) {
    // Humidity from SHT40
    auto* inside_humidity =
        new RepeatSensor<float>(read_interval, read_humid_callback);

    auto inside_humidity_sk_output = new SKOutputFloat(
        "environment.inside.relativeHumidity", "/insideHumidity/skPath",
        new SKMetadata("ratio", "Inside Humidity"));
    ConfigItem(inside_humidity_sk_output)
        ->set_title("Inside Humidity Signal K Path")
        ->set_description("Signal K path for the humidity inside the boat")
        ->set_sort_order(500);
    inside_humidity->connect_to(inside_humidity_sk_output);
  }

  ///////////////////////////////////////////////////////////////////
  // Analog inputs

  bool enable_signalk_output = true;

  // Fuel tank sender on ADS1115 channel 3 (A4 on the silkscreen).
  ConnectTankSender(ads1115, 3, "Fuel", "fuel.0", 3000, enable_signalk_output);

  // Read the voltage level of another analog input, e.g. A2 (channel 1):
  //
  // auto a2_voltage = new ADS1115VoltageInput(ads1115, 1, "/Voltage A2");
  //
  // ConfigItem(a2_voltage)
  //     ->set_title("Analog Voltage A2")
  //     ->set_description("Voltage level of analog input A2")
  //     ->set_sort_order(3000);
  //
  // a2_voltage->connect_to(
  //     new SKOutputFloat("sensors.a2.voltage", "/Voltage A2/skPath",
  //                        new SKMetadata("V", "Analog Voltage A2")));
  //
  // To convert the voltage into something else (e.g. a distance with a
  // conversion factor of 0.17 m/V), insert a Linear transform in between:
  //
  // auto a2_distance = new Linear(0.17, 0.0, "/Voltage A2/linear");
  // a2_voltage->connect_to(a2_distance)
  //     ->connect_to(new SKOutputFloat("sensors.a2.distance",
  //                                     "/Distance A2/skPath",
  //                                     new SKMetadata("m", "Analog Distance A2")));

  ///////////////////////////////////////////////////////////////////
  // Digital tacho input
  //
  // ConnectTachoSender() (see halmet_digital.cpp) already wires the
  // frequency directly to a Signal K output - no NMEA 2000 sender needed.
  // multiplier for Paikea: 0.11400
  ConnectTachoSender(kDigitalInputPin4, "0");

  ///////////////////////////////////////////////////////////////////
  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
