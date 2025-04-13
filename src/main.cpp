// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifdef ENABLE_NMEA2000_OUTPUT
#include <NMEA2000_esp32.h>
#endif

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

#ifdef ENABLE_SIGNALK
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder
#else
#include "sensesp_minimal_app_builder.h"
#endif

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"

#include "sensesp/transforms/time_counter.h" // voor timecounter runtime engine


using namespace sensesp;
using namespace halmet;

#ifndef ENABLE_SIGNALK
#define BUILDER_CLASS SensESPMinimalAppBuilder
SensESPMinimalApp* sensesp_app;
Networking* networking;
MDNSDiscovery* mdns_discovery;
HTTPServer* http_server;
SystemStatusLed* system_status_led;
#endif

//// ONE WIRE
#ifdef ENABLE_ONE_WIRE
#include "sensesp_onewire/onewire_temperature.h"
using namespace sensesp::onewire;
const int OneWirePin = 4;
uint read_delay = 500;
#endif

/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.

#ifdef ENABLE_NMEA2000_OUTPUT
tNMEA2000* nmea2000;
elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;
#endif

TwoWire* i2c;
Adafruit_SSD1306* display;

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

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
// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.


// Kees: with 13 setup as ratio in halmet_digital.cpp
// 49 Simulates ~228 RPM
// 217 Simulates ~1000 RPM
// 325 Simulates ~1500 RPM

const int kTestOutputFrequency = 217;


#endif


/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // These calls can be used for fine-grained control over the logging level.
  // esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);

  Serial.begin(115200);

  /////////////////////////////////////////////////////////////////////
  // Initialize the application framework

  // Construct the global SensESPApp() object
  BUILDER_CLASS builder;
  sensesp_app = (&builder)
                    // EDIT: Set a custom hostname for the app.
                    ->set_hostname("halmet")
                    // EDIT: Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("***", "***")
                    //->set_sk_server("192.168.1.251", 3000)
                    // EDIT: Enable OTA updates with a password.
                    ->enable_ota("otakees")
                    ->get_app();


  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();

  ads1115->setGain(kADS1115Gain);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  // Set the LEDC peripheral to a 13-bit resolution
  ledcSetup(0, kTestOutputFrequency, 13);
  // Attach the channel to the GPIO pin to be controlled
  ledcAttachPin(kTestOutputPin, 0);
  // Set the duty cycle to 50%
  // Duty cycle value is calculated based on the resolution
  // For 13-bit resolution, max value is 8191, so 50% is 4096
  ledcWrite(0, 4096);
#endif

#ifndef ENABLE_SIGNALK
  // Initialize components that would normally be present in SensESPApp
  networking = new Networking("/System/WiFi Settings", "", "");
  ConfigItem(networking);
  mdns_discovery = new MDNSDiscovery();
  http_server = new HTTPServer();
  system_status_led = new SystemStatusLed(LED_BUILTIN);
#endif


  // Initialize the OLED display
  bool display_present = InitializeSSD1306(sensesp_app.get(), &display, i2c);

  ///////////////////////////////////////////////////////////////////
  // Analog inputs

#ifdef ENABLE_SIGNALK
  bool enable_signalk_output = true;
#else
  bool enable_signalk_output = false;
#endif

  // Connect the tank senders.
  // EDIT: To enable more tanks, uncomment the lines below.
  auto tank_a1_volume = ConnectTankSender(ads1115, 0, "Fuel", "fuel.main", 3000,
                                          enable_signalk_output); // A1 channel is 0
  // auto tank_a2_volume = ConnectTankSender(ads1115, 1, "A2");
  // auto tank_a3_volume = ConnectTankSender(ads1115, 2, "A3");
  // auto tank_a4_volume = ConnectTankSender(ads1115, 3, "A4");

  // Connect the temperature senders.
  auto temperature_a3_kelvin = ConnectTemperatureSensor(ads1115, 2, "Coolant", "propulsion.main.coolantTemperature", 3000, // Geen idee of dit klopt, hier mimic een soort tank setup maar het is temperature.
    enable_signalk_output); // A3 channel is 2

  // Connect the oil pressure senders
  auto oilpressure_a4_bar = ConnectOilPressureSensor(ads1115, 3, "Oil", "main", 3000, // Geen idee of dit klopt, hier mimic een soort tank setup maar het is oil pressure.
    enable_signalk_output); // A4 channel is 3

#ifdef ENABLE_NMEA2000_OUTPUT

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


  // Tank 1, instance 0. Capacity 200 liters. You can change the capacity
  // in the web UI as well.
  // EDIT: Make sure this matches your tank configuration above.
  // Standaard Tank van Sun Odyssey 32 is 70L
  N2kFluidLevelSender* tank_a1_sender = new N2kFluidLevelSender(
      "/Tanks/Fuel/NMEA 2000", 0, N2kft_Fuel, 70, nmea2000);

  ConfigItem(tank_a1_sender)
      ->set_title("Diesel Tank A1 NMEA 2000")
      ->set_description("NMEA 2000 tank sender for dieseltank A1")
      ->set_sort_order(3005);

  tank_a1_volume->connect_to(&(tank_a1_sender->tank_level_));

#endif  // ENABLE_NMEA2000_OUTPUT

  // Read the voltage level of analog input A2
  auto a2_voltage = new ADS1115VoltageInput(ads1115, 1, "/Voltage A2");

  ConfigItem(a2_voltage)
      ->set_title("Analog Voltage A2")
      ->set_description("Voltage level of analog input A2")
      ->set_sort_order(3000);

  a2_voltage->connect_to(new LambdaConsumer<float>(
      [](float value) { debugD("Voltage A2: %f", value); }));

  // If you want to output something else than the voltage value,
  // you can insert a suitable transform here.
  // For example, to convert the voltage to a distance with a conversion
  // factor of 0.17 m/V, you could use the following code:
  // auto a2_distance = new Linear(0.17, 0.0);
  // a2_voltage->connect_to(a2_distance);

#ifdef ENABLE_SIGNALK
  a2_voltage->connect_to(
      new SKOutputFloat("propulsion.main.alternatorVoltage", "Analog Voltage A2", // origineel was "sensors.a2.voltage", "Analog Voltage A2"
                        new SKMetadata("V","Analog Voltage A2")));
  // Example of how to output the distance value to Signal K.
  // a2_distance->connect_to(
  //     new SKOutputFloat("sensors.a2.distance", "Analog Distance A2",
  //                       new SKMetadata("m", "Analog Distance A2")));

#endif

  ///////////////////////////////////////////////////////////////////
  // Digital alarm inputs

  // EDIT: More alarm inputs can be defined by duplicating the lines below.
  // Make sure to not define a pin for both a tacho and an alarm.
  auto alarm_d2_input = ConnectAlarmSender(kDigitalInputPin2, "D2"); // low oil pressure alarm
  auto alarm_d3_input = ConnectAlarmSender(kDigitalInputPin3, "D3"); // engine high temp alarm
  auto alarm_d4_input = ConnectAlarmSender(kDigitalInputPin4, "D4"); // bilge alarm

  // Update the alarm states based on the input value changes.
  // EDIT: If you added more alarm inputs, uncomment the respective lines below.
  alarm_d2_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  // In this example, alarm_d3_input is active low, so invert the value.
  auto alarm_d3_inverted = alarm_d3_input->connect_to(
      new LambdaTransform<bool, bool>([](bool value) { return !value; }));
  alarm_d3_inverted->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  alarm_d4_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));

  // Connect the tacho senders. Engine name is "main".
  // EDIT: More tacho inputs can be defined by duplicating the line below.
  auto tacho_d1_frequency = ConnectTachoSender(kDigitalInputPin1, "main");


  // Connect outputs to the N2k senders.
#ifdef ENABLE_NMEA2000_OUTPUT
  // EDIT: This example connects the D2 alarm input to the low oil pressure
  // warning. Modify according to your needs.
  N2kEngineParameterDynamicSender* engine_dynamic_sender =
      new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0,
                                          nmea2000);

  ConfigItem(engine_dynamic_sender)
      ->set_title("Engine 1 Dynamic")
      ->set_description("NMEA 2000 dynamic engine parameters for engine 1")
      ->set_sort_order(3010);

  alarm_d2_input->connect_to(engine_dynamic_sender->low_oil_pressure_);

  // This is just an example -- normally temperature alarms would not be
  // active-low (inverted).
  alarm_d3_inverted->connect_to(engine_dynamic_sender->over_temperature_);


  // Coolant Temperature send
  temperature_a3_kelvin->connect_to(engine_dynamic_sender->temperature_);

  //Oil pressure conversion from bar to kPa then send to nmea
  auto oilpressure_a4_kpa = new Linear(100.0,0.0);  // bar → kPa
    oilpressure_a4_bar->connect_to(oilpressure_a4_kpa)->connect_to(engine_dynamic_sender->oil_pressure_);

  // EDIT: Make sure this matches your tacho configuration above.
  //       Duplicate the lines below to connect more tachos, but be sure to
  //       use different engine instances
  N2kEngineParameterRapidSender* engine_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 1 Rapid Update", 0,
                                        nmea2000);  // Engine 1, instance 0

  ConfigItem(engine_rapid_sender)
      ->set_title("Engine 1 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 1")
      ->set_sort_order(3015);

  tacho_d1_frequency->connect_to(&(engine_rapid_sender->engine_speed_));


 // Initialize the N2kBilgeAlarmSender with appropriate values
  String bilge_config_path = "/bilge_alarm";  // Or use the path where the config is stored
  uint8_t bilge_instance = 1;  // Set a unique instance ID
  bool initial_alarm_state = alarm_d4_input->get();  // Start with the initial state

  // Create N2kBilgeAlarmSender instance
  N2kBilgeAlarmSender* bilge_alarm_sender = new N2kBilgeAlarmSender(bilge_config_path, bilge_instance, initial_alarm_state, nmea2000);

  alarm_d4_input->connect_to(bilge_alarm_sender->alarm_state_);


#endif 



//////////// ONE WIRE
  #ifdef ENABLE_ONE_WIRE
    DallasTemperatureSensors* dts = new DallasTemperatureSensors(OneWirePin);

    // Measure temperature 1
    auto probe_1_temp = new OneWireTemperature(dts, read_delay, "/exhaustTemperature/oneWire");

    ConfigItem(probe_1_temp)
      ->set_title("1Wire Temp 1")
      ->set_description("Temp from 1 Wire sensor #1")
      ->set_sort_order(3100);

    probe_1_temp->connect_to(new LambdaConsumer<float>(
        [](float value) { debugD("Temp T1: %f", value); }));

    // SEND NMEA One wire

    #ifdef ENABLE_NMEA2000_OUTPUT

    // Create N2kExhaustTemperatureSender with appropriate values
    String exhaust_config_path = "/exhaust_temp";  // Use the path where the config is stored
    uint8_t exhaust_instance = 1;                  // Unique instance ID for the exhaust probe

    // Create the N2kExhaustTemperatureSender instance
    N2kExhaustTemperatureSender* exhaust_temp_sender = new N2kExhaustTemperatureSender(exhaust_config_path, exhaust_instance, 0.0, nmea2000);

    // Connect the temperature data producer to the N2kExhaustTemperatureSender
    probe_1_temp->connect_to(exhaust_temp_sender->temperature_);

    #endif

    #ifdef ENABLE_SIGNALK
      probe_1_temp->connect_to(
          new SKOutputFloat("propulsion.main.exhaustTemperature", "1",new SKMetadata("K","1Wire Temp Value T1"))
        );
    #endif
  #endif


  // calculate engine runtime en motor state
 // Create a frequency transform

  // create a propulsion state lambda transform
  auto* propulsion_state = new LambdaTransform<float, String>(
    [](bool freq) {
      if (freq > 0) {
        return "started";
      } else {
        return "stopped";
      }
    },
    "/Transforms/Propulsion State");

ConfigItem(propulsion_state)
    ->set_title("Propulsion State")
    ->set_description("Propulsion State Description")
    ->set_sort_order(1200);

tacho_d1_frequency->connect_to(propulsion_state);

// create engine hours counter using PersistentDuration
auto* engine_hours = new TimeCounter<float>("/Transforms/Engine Hours");

ConfigItem(engine_hours)
    ->set_title("Engine Hours")
    ->set_description("Engine Hours Description")
    ->set_sort_order(1300);

tacho_d1_frequency->connect_to(engine_hours);


// Note; voor NMEA moet ik het nog fixen dat hij alleen maar uren stuurt. Dus dan engine_hours_offset delen door 3600.

#ifdef ENABLE_SIGNALK
// create and connect the propulsion state output object
propulsion_state->connect_to(new SKOutput<String>(
    "propulsion.main.state", "", new SKMetadata("", "Main Engine State")));

// Connect engine_hours to the offset transform
engine_hours->connect_to(new SKOutput<float>("propulsion.main.runTime", "",
                                  new SKMetadata("s", "Main Engine running time")));
#endif


#ifdef ENABLE_NMEA2000_OUTPUT

// send through NMEA2000 the total engine hours

auto engine_hours_in_hours = engine_hours->connect_to(
    new LambdaTransform<float, float>([](float seconds) { return seconds / 3600.0f; }));

  engine_hours_in_hours
      ->connect_to(engine_dynamic_sender->total_engine_hours_);  // Send converted value to NMEA

#endif

  ///////////////////////////////////////////////////////////////////
  // Display setup

  // Connect the outputs to the display
  if (display_present) {
#ifdef ENABLE_SIGNALK
    event_loop()->onRepeat(1000, []() {
     PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });
#endif

// Display tank level
// EDIT: Duplicate the lines below to make the display show all your tanks.

    tank_a1_volume->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 2, "Diesel Tank A1", 100 * value); }));

// Display RPM
    tacho_d1_frequency->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 3, "RPM D1", 60 * value); }));
        // note the '60' here is because it's measured in Hz and converting Hz to RPM is 60

/*
// Create a poor man's "christmas tree" display for the alarms
    event_loop()->onRepeat(1000, []() {
      char state_string[5] = {};
      for (int i = 0; i < 4; i++) {
        state_string[i] = alarm_states[i] ? '*' : '_';
      }
      PrintValue(display, 4, "Alarm", state_string);
    });
*/

// Display voltage A2
    a2_voltage->connect_to(new LambdaConsumer<float>(
      [](float value) { PrintValue(display, 4, "A2 Voltage", value); }));

// Display Temp A3 - resistive sensor coolant
temperature_a3_kelvin->connect_to(new LambdaConsumer<float>(
  [](float value) { PrintValue(display, 5, "A3 Kelvin", value); }));

// Display Temp T1 - onewire temp exhaust
    probe_1_temp->connect_to(new LambdaConsumer<float>(
      [](float value) { PrintValue(display, 6, "T1 Kelvin", value); }));

  }

// Display Engine Hours
engine_hours_in_hours->connect_to(new LambdaConsumer<float>(
  [](float value) { PrintValue(display, 7, "Engine Hours", value); }));


  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}



void loop() { event_loop()->tick(); }
