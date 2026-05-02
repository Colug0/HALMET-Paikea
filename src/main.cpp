// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.
#define ENABLE_SIGNALK

#include <Adafruit_ADS1X15.h>
#include <NMEA2000_esp32.h>

#include "n2k_senders.h"
#include "sensesp/net/discovery.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/system_status_led.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_serial.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "M5UnitENV.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/wiring.h"



using namespace sensesp;
using namespace halmet;
using namespace sensesp::onewire;
using namespace sensesp::nmea0183;

/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.

tNMEA2000* nmea2000;
elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

TwoWire* i2c;

DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);
SHT4X sht4;
BMP280 bmp;



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
// Callback funktions for i2c sensors
float read_temp_callback() { return (bmp.readTemperature() + 273.15); } //convert value to Kelvin
float read_press_callback() { return (bmp.readPressure()); }
float read_humid_callback() { float humidityValue;  sht4.update();   humidityValue = (sht4.humidity)/100;  return (humidityValue); }


/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
  //SetupLogging(ESP_LOG_DEBUG);
  //SetupLogging(ESP_LOG_NONE);
  SetupLogging(ESP_LOG_VERBOSE);
  
  // Define how often SensESP should read the sensor(s) in milliseconds
  uint read_delay = 1000; //Intervall for OneWire Sensors
  unsigned int read_interval = 2000; // Intervall for environment Sensors (Temp, Humid and Baro)

  // These calls can be used for fine-grained control over the logging level.
  // esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);

  Serial.begin(115200);

  /////////////////////////////////////////////////////////////////////
  wifi_auth_mode_t auth = WIFI_AUTH_WPA_PSK;
  WiFi.setMinSecurity(auth);
  // Initialize the application framework

  // Construct the global SensESPApp() object
  BUILDER_CLASS builder;
  sensesp_app = (&builder)
                    ->set_hostname("halmet")
                    ->set_wifi("Paikea", "2001BestesBootderWelt!")
                    //->set_sk_server("192.168.88.100", 3000)
                    ->set_sk_server("halos.local", 4430)
                    // EDIT: Enable OTA updates with a password.
                    ->enable_ota("!HalmetSecretWiFiOTApass")
                    ->get_app();

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();

  ads1115->setGain(kADS1115Gain);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20231229",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "HALMET",    // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    71  // Default N2k node address
  );
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  //////////////////////////////////////////
  
  // OneWire
  // Measure refrigerator temperature
  
  auto refrigerator_temp =
      new OneWireTemperature(dts, read_delay, "/refrigeratorTemperature/oneWire");

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

  auto refrigerator_temp_sk_output = new SKOutputFloat(
      "environment.inside.refrigerator.temperature", "/refrigeratorTemperature/skPath");

  ConfigItem(refrigerator_temp_sk_output)
      ->set_title("refrigerator Temperature Signal K Path")
      ->set_description("Signal K path for the refrigerator temperature")
      ->set_sort_order(300);

  refrigerator_temp->connect_to(refrigerator_temp_calibration)
      ->connect_to(refrigerator_temp_sk_output);

// Measure engine temperature     
auto engine_temp =
      new OneWireTemperature(dts, read_delay, "/engineTemperature/oneWire");

  ConfigItem(engine_temp)
      ->set_title("engine Temperature(alternator)")
      ->set_description("Temperature of the alternator on the engine")
      ->set_sort_order(100);

  auto engine_temp_calibration =
      new Linear(1.0, 0.0, "/engineTemperature/linear");

  ConfigItem(engine_temp_calibration)
      ->set_title("engine Temperature Calibration")
      ->set_description("Calibration for the engine temperature sensor")
      ->set_sort_order(200);

  auto engine_temp_sk_output = new SKOutputFloat(
      "propulsion.0.temperature", "/engineTemperature/skPath");
      
  ConfigItem(engine_temp_sk_output)
      ->set_title("engine Temperature Signal K Path")
      ->set_description("Signal K path for the engine temperature")
      ->set_sort_order(300);

  engine_temp->connect_to(engine_temp_calibration)
      ->connect_to(engine_temp_sk_output);

// Measure alternator temperature
  auto alternator_temp =
      new OneWireTemperature(dts, read_delay, "/alternatorTemperature/oneWire");

  ConfigItem(alternator_temp)
      ->set_title("alternator Temperature")
      ->set_description("Temperature of the alternator")
      ->set_sort_order(100);

  auto alternator_temp_calibration =
      new Linear(1.0, 0.0, "/alternatorTemperature/linear");

  ConfigItem(alternator_temp_calibration)
      ->set_title("alternator Temperature Calibration")
      ->set_description("Calibration for the alternator temperature sensor")
      ->set_sort_order(200);

  auto alternator_temp_sk_output = new SKOutputFloat(
      "electrical.alternator.temperature", "/AlternatorTemperatureTemperature/skPath");

  ConfigItem(alternator_temp_sk_output)
      ->set_title("alternator Temperature Signal K Path")
      ->set_description("Signal K path for the alternator temperature")
      ->set_sort_order(300);   

  alternator_temp->connect_to(alternator_temp_calibration)
      ->connect_to(alternator_temp_sk_output);

// I2C Sensor setup
  // BMP280 -> Temperature and Pressure
    if (!bmp.begin(i2c, BMP280_I2C_ADDR, 21, 22, 400000U)) {
        Serial.println("Couldn't find BMP280");
        while (1) delay(1);
    }
    bmp.setSampling(BMP280::MODE_NORMAL,     // Operating Mode. 
                    BMP280::SAMPLING_X2,     // Temp. oversampling 
                    BMP280::SAMPLING_X16,    // Pressure oversampling 
                    BMP280::FILTER_X16,      // Filtering. 
                    BMP280::STANDBY_MS_500); // Standby time. 
  // SHT40 -> Humidity
    if (!sht4.begin(i2c, SHT40_I2C_ADDR_44, 21, 22, 400000U)) {
        Serial.println("Couldn't find SHT4x");
        while (1) delay(1);
    }
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);

  // Temperatur from bmp280
  auto* inside_temp = new RepeatSensor<float>(read_interval, read_temp_callback);

  auto inside_temp_sk_output = new SKOutputFloat(
      "environment.inside.temperature", "/insideTemperature/skPath");

  ConfigItem(inside_temp_sk_output)
      ->set_title("inside Temperature Signal K Path")
      ->set_description("Signal K path for the Temperature inside of the boat")
      ->set_sort_order(300);

  inside_temp->connect_to(inside_temp_sk_output);

  // Pressure from bmp280
  auto* inside_pressure = new RepeatSensor<float>(read_interval, read_press_callback);

  auto inside_pressure_sk_output = new SKOutputFloat(
      "environment.inside.pressure", "/insidePressure/skPath");
  ConfigItem(inside_pressure_sk_output)
      ->set_title("inside Pressure Signal K Path")
      ->set_description("Signal K path for the Pressure inside of the boat")
      ->set_sort_order(400);
  inside_pressure->connect_to(inside_pressure_sk_output);

  // Humidity from SHT40
  auto* inside_humidity = new RepeatSensor<float>(read_interval, read_humid_callback);

  auto inside_humidity_sk_output = new SKOutputFloat(
      "environment.inside.relativeHumidity", "/insideHumidity/skPath");
  ConfigItem(inside_humidity_sk_output)
      ->set_title("inside Humidity Signal K Path")  
      ->set_description("Signal K path for the Humidity inside of the boat")
      ->set_sort_order(500);
  inside_humidity->connect_to(inside_humidity_sk_output);


  // GNSS

  HardwareSerial* serial = &Serial1;
  serial->begin(kGNSSBitRate, SERIAL_8N1, kGNSSRxPin, kGNSSTxPin);

  NMEA0183IOTask* nmea0183_io_task = new NMEA0183IOTask(serial);

  ConnectGNSS(&nmea0183_io_task->parser_, new GNSSData());

  //event_loop()->onAvailable(Serial1, [](){Serial.write(Serial1.read());  });
//while (Serial1.available()) {
//    char c = Serial1.read();
//    Serial.write(c); // Gibt das Zeichen direkt weiter
//  }
  
  ///////////////////////////////////////////////////////////////////
  // Analog inputs
  
  bool enable_signalk_output = true;

  // Connect the tank senders.
  auto tank_a4_volume = ConnectTankSender(ads1115, 3, "Fuel", "fuel.0", 3000,
                                          enable_signalk_output);


#ifdef ENABLE_NMEA2000_OUTPUT
  // Fuel Tank, instance 0. Capacity 150 liters. 
  N2kFluidLevelSender* tank_a4_sender = new N2kFluidLevelSender(
      "/Tanks/Fuel/NMEA 2000", 0, N2kft_Fuel, 150, nmea2000);

  ConfigItem(tank_a4_sender)
      ->set_title("Tank A4 NMEA 2000")
      ->set_description("NMEA 2000 tank sender for tank A4")
      ->set_sort_order(3005);

  tank_a4_volume->connect_to(&(tank_a4_sender->tank_level_));
#endif  // ENABLE_NMEA2000_OUTPUT


  // Read the voltage level of analog input A2
   //example auto a2_voltage = new ADS1115VoltageInput(ads1115, 1, "/Voltage A2");

   //example ConfigItem(a2_voltage)
    //example    ->set_title("Analog Voltage A2")
    //example    ->set_description("Voltage level of analog input A2")
   //example     ->set_sort_order(3000);

   //example a2_voltage->connect_to(new LambdaConsumer<float>(
  //example      [](float value) { debugD("Voltage A2: %f", value); }));

  // If you want to output something else than the voltage value,
  // you can insert a suitable transform here.
  // For example, to convert the voltage to a distance with a conversion
  // factor of 0.17 m/V, you could use the following code:
  // auto a2_distance = new Linear(0.17, 0.0);
  // a2_voltage->connect_to(a2_distance);

  //example a2_voltage->connect_to(
  //example     new SKOutputFloat("sensors.a2.voltage", "Analog Voltage A2",
  //example                       new SKMetadata("V", "Analog Voltage A2")));
  // Example of how to output the distance value to Signal K.
  // a2_distance->connect_to(
  //     new SKOutputFloat("sensors.a2.distance", "Analog Distance A2",
  //                       new SKMetadata("m", "Analog Distance A2")));



  ///////////////////////////////////////////////////////////////////
  // Digital tacho inputs

  // Connect the tacho senders. Engine name is "0".
  auto tacho_d4_frequency = ConnectTachoSender(kDigitalInputPin4, "0");

  // Connect outputs to the N2k senders.
  N2kEngineParameterRapidSender* engine_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 0 Rapid Update", 0,
                                        nmea2000);  // Engine 0, instance 0

  ConfigItem(engine_rapid_sender)
      ->set_title("Engine 0 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 0")
      ->set_sort_order(3015);

  tacho_d4_frequency->connect_to(&(engine_rapid_sender->engine_speed_));

  ///////////////////////////////////////////////////////////////////


  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
