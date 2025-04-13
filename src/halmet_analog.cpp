#include "halmet_analog.h"

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/ui/config_item.h"

namespace halmet {

// HALMET constant measurement current (A)
const float kMeasurementCurrent = 0.01;


// Default fuel tank size, in m3
const float kTankDefaultSize = 120. / 1000;

// --- Tank Sensor Code ---
sensesp::FloatProducer* ConnectTankSender(Adafruit_ADS1115* ads1115,
                                          int channel, const String& name,
                                          const String& sk_id, int sort_order,
                                          bool enable_signalk_output) {
  const uint ads_read_delay = 500;  // ms

  // Configure the sender resistance sensor

  auto sender_resistance =
      new sensesp::RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return kVoltageDividerScale * adc_output_volts / kMeasurementCurrent;
      });

  if (enable_signalk_output) {
    char resistance_sk_config_path[80];
    snprintf(resistance_sk_config_path, sizeof(resistance_sk_config_path),
             "/Tanks/%s/Resistance/SK Path", name.c_str());
    char resistance_title[80];
    snprintf(resistance_title, sizeof(resistance_title),
             "%s Tank Sender Resistance SK Path", name.c_str());
    char resistance_description[80];
    snprintf(resistance_description, sizeof(resistance_description),
             "Signal K path for the sender resistance of the %s tank",
             name.c_str());
    char resistance_sk_path[80];
    snprintf(resistance_sk_path, sizeof(resistance_sk_path),
             "tanks.%s.senderResistance", sk_id.c_str());
    char resistance_meta_display_name[80];
    snprintf(resistance_meta_display_name, sizeof(resistance_meta_display_name),
             "Resistance %s", name.c_str());
    char resistance_meta_description[80];
    snprintf(resistance_meta_description, sizeof(resistance_meta_description),
             "Measured tank %s sender resistance", name.c_str());

    auto sender_resistance_sk_output = new sensesp::SKOutputFloat(
        resistance_sk_path, resistance_sk_config_path,
        new sensesp::SKMetadata("ohm", resistance_meta_display_name,
                                resistance_meta_description));

    ConfigItem(sender_resistance_sk_output)
        ->set_title(resistance_title)
        ->set_description(resistance_description)
        ->set_sort_order(sort_order);

    sender_resistance->connect_to(sender_resistance_sk_output);
  }

  // Configure the piecewise linear interpolator for the tank level (ratio)

  char curve_config_path[80];
  snprintf(curve_config_path, sizeof(curve_config_path),
           "/Tanks/%s/Level Curve", name.c_str());
  char curve_title[80];
  snprintf(curve_title, sizeof(curve_title), "%s Tank Level Curve",
           name.c_str());
  char curve_description[80];
  snprintf(curve_description, sizeof(curve_description),
           "Piecewise linear curve for the %s tank level", name.c_str());

  auto tank_level = (new sensesp::CurveInterpolator(nullptr, curve_config_path))
                        ->set_input_title("Sender Resistance (ohms)")
                        ->set_output_title("Fuel Level (ratio)");

  ConfigItem(tank_level)
      ->set_title(curve_title)
      ->set_description(curve_description)
      ->set_sort_order(sort_order + 1);

  if (tank_level->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    // heb van een site genomen
    tank_level->clear_samples();
    tank_level->add_sample(sensesp::CurveInterpolator::Sample(0, 0));
    tank_level->add_sample(sensesp::CurveInterpolator::Sample(95., 0.5));
    tank_level->add_sample(sensesp::CurveInterpolator::Sample(190., 1));
  }

  sender_resistance->connect_to(tank_level);

  if (enable_signalk_output) {
    char level_config_path[80];
    snprintf(level_config_path, sizeof(level_config_path),
             "/Tanks/%s/Current Level SK Path", name.c_str());
    char level_title[80];
    snprintf(level_title, sizeof(level_title), "%s Tank Level SK Path",
             name.c_str());
    char level_description[80];
    snprintf(level_description, sizeof(level_description),
             "Signal K path for the %s tank level", name.c_str());
    char level_sk_path[80];
    snprintf(level_sk_path, sizeof(level_sk_path), "tanks.%s.currentLevel",
             sk_id.c_str());
    char level_meta_display_name[80];
    snprintf(level_meta_display_name, sizeof(level_meta_display_name),
             "Tank %s level", name.c_str());
    char level_meta_description[80];
    snprintf(level_meta_description, sizeof(level_meta_description),
             "Tank %s level", name.c_str());

    auto tank_level_sk_output = new sensesp::SKOutputFloat(
        level_sk_path, level_config_path,
        new sensesp::SKMetadata("ratio", level_meta_display_name,
                                level_meta_description));

    ConfigItem(tank_level_sk_output)
        ->set_title(level_title)
        ->set_description(level_description)
        ->set_sort_order(sort_order + 2);

    tank_level->connect_to(tank_level_sk_output);
  }

  // Configure the linear transform for the tank volume

  char volume_config_path[80];
  snprintf(volume_config_path, sizeof(volume_config_path),
           "/Tanks/%s/Total Volume", name.c_str());
  char volume_title[80];
  snprintf(volume_title, sizeof(volume_title), "%s Tank Total Volume",
           name.c_str());
  char volume_description[80];
  snprintf(volume_description, sizeof(volume_description),
           "Calculated total volume of the %s tank", name.c_str());
  auto tank_volume =
      new sensesp::Linear(kTankDefaultSize, 0, volume_config_path);

  ConfigItem(tank_volume)
      ->set_title(volume_title)
      ->set_description(volume_description)
      ->set_sort_order(sort_order + 3);

  tank_level->connect_to(tank_volume);

  if (enable_signalk_output) {
    char volume_sk_config_path[80];
    snprintf(volume_sk_config_path, sizeof(volume_sk_config_path),
             "/Tanks/%s/Current Volume SK Path", name.c_str());
    char volume_title[80];
    snprintf(volume_title, sizeof(volume_title), "%s Tank Volume SK Path",
             name.c_str());
    char volume_description[80];
    snprintf(volume_description, sizeof(volume_description),
             "Signal K path for the %s tank volume", name.c_str());
    char volume_sk_path[80];
    snprintf(volume_sk_path, sizeof(volume_sk_path), "tanks.%s.currentVolume",
             sk_id.c_str());
    char volume_meta_display_name[80];
    snprintf(volume_meta_display_name, sizeof(volume_meta_display_name),
             "Tank %s volume", name.c_str());
    char volume_meta_description[80];
    snprintf(volume_meta_description, sizeof(volume_meta_description),
             "Calculated tank %s remaining volume", name.c_str());

    auto tank_volume_sk_output = new sensesp::SKOutputFloat(
        volume_sk_path, volume_sk_config_path,
        new sensesp::SKMetadata("m3", volume_meta_display_name,
                                volume_meta_description));

    ConfigItem(tank_volume_sk_output)
        ->set_title(volume_title)
        ->set_description(volume_description)
        ->set_sort_order(sort_order + 4);

    tank_volume->connect_to(tank_volume_sk_output);
  }

  return tank_level;
}

// --- Temperature Sensor Code ---
sensesp::FloatProducer* ConnectTemperatureSensor(Adafruit_ADS1115* ads1115,
                                                 int channel, const String& name,
                                                 const String& sk_id, int sort_order,
                                                 bool enable_signalk_output) {
  const uint ads_read_delay = 500;  // ms

  // Configure the temperature resistance sensor
  auto temperature_resistance =
      new sensesp::RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return kVoltageDividerScale * adc_output_volts / kMeasurementCurrent;
      });

  if (enable_signalk_output) {
    char resistance_sk_config_path[80];
    snprintf(resistance_sk_config_path, sizeof(resistance_sk_config_path),
             "/Temperature/%s/Resistance/SK Path", name.c_str());
    char resistance_title[80];
    snprintf(resistance_title, sizeof(resistance_title),
             "%s Temperature Sensor Resistance SK Path", name.c_str());
    char resistance_description[80];
    snprintf(resistance_description, sizeof(resistance_description),
             "Signal K path for the sensor resistance of the %s temperature sensor",
             name.c_str());
    char resistance_sk_path[80];
    snprintf(resistance_sk_path, sizeof(resistance_sk_path),
             "%s.sensorResistance", sk_id.c_str());                                // was dit "temperature.%s.sensorResistance"
    char resistance_meta_display_name[80];
    snprintf(resistance_meta_display_name, sizeof(resistance_meta_display_name),
             "Resistance %s", name.c_str());
    char resistance_meta_description[80];
    snprintf(resistance_meta_description, sizeof(resistance_meta_description),
             "Measured temperature %s sensor resistance", name.c_str());

    auto temperature_resistance_sk_output = new sensesp::SKOutputFloat(
        resistance_sk_path, resistance_sk_config_path,
        new sensesp::SKMetadata("ohm", resistance_meta_display_name,
                                resistance_meta_description));

    ConfigItem(temperature_resistance_sk_output)
        ->set_title(resistance_title)
        ->set_description(resistance_description)
        ->set_sort_order(sort_order);

    temperature_resistance->connect_to(temperature_resistance_sk_output);
  }

  // Configure the piecewise linear interpolator for temperature in Kelvin
  char curve_config_path[80];
  snprintf(curve_config_path, sizeof(curve_config_path),
           "/Temperature/%s/Curve", name.c_str());
  char curve_title[80];
  snprintf(curve_title, sizeof(curve_title), "%s Temperature Curve",
           name.c_str());
  char curve_description[80];
  snprintf(curve_description, sizeof(curve_description),
           "Piecewise linear curve for the %s temperature sensor", name.c_str());

  auto temperature_kelvin = (new sensesp::CurveInterpolator(nullptr, curve_config_path))
                                ->set_input_title("Sensor Resistance (ohms)")
                                ->set_output_title("Temperature (K)");

  ConfigItem(temperature_kelvin)
      ->set_title(curve_title)
      ->set_description(curve_description)
      ->set_sort_order(sort_order + 1);

  if (temperature_kelvin->get_samples().empty()) {
    // Provide a default calibration curve
    temperature_kelvin->clear_samples();
    temperature_kelvin->add_sample(sensesp::CurveInterpolator::Sample(450, 298.15)); // 25°C in K
    temperature_kelvin->add_sample(sensesp::CurveInterpolator::Sample(23, 393.15));  // 120°C in K
  }

  temperature_resistance->connect_to(temperature_kelvin);

  if (enable_signalk_output) {
    char temperature_config_path[80];
    snprintf(temperature_config_path, sizeof(temperature_config_path),
             "/Temperature/%s/Current Temperature SK Path", name.c_str());
    char temperature_title[80];
    snprintf(temperature_title, sizeof(temperature_title), "%s Temperature SK Path",
             name.c_str());
    char temperature_description[80];
    snprintf(temperature_description, sizeof(temperature_description),
             "Signal K path for the %s temperature", name.c_str());
    char temperature_sk_path[80];
    snprintf(temperature_sk_path, sizeof(temperature_sk_path), "%s", // was dit "temperature.%s.currentTemperature"
             sk_id.c_str());
    char temperature_meta_display_name[80];
    snprintf(temperature_meta_display_name, sizeof(temperature_meta_display_name),
             "Temperature %s", name.c_str());
    char temperature_meta_description[80];
    snprintf(temperature_meta_description, sizeof(temperature_meta_description),
             "Measured temperature in Kelvin for %s", name.c_str());

    auto temperature_sk_output = new sensesp::SKOutputFloat(
        temperature_sk_path, temperature_config_path,
        new sensesp::SKMetadata("K", temperature_meta_display_name,
                                temperature_meta_description));

    ConfigItem(temperature_sk_output)
        ->set_title(temperature_title)
        ->set_description(temperature_description)
        ->set_sort_order(sort_order + 2);

    temperature_kelvin->connect_to(temperature_sk_output);
  }

  return temperature_kelvin;
}

sensesp::FloatProducer* ConnectOilPressureSensor(Adafruit_ADS1115* ads1115,
  int channel, const String& name,
  const String& sk_id, int sort_order,
  bool enable_signalk_output) {
  const uint ads_read_delay = 500;  // ms

  auto resistance_sensor =
    new sensesp::RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
    int16_t adc_output = ads1115->readADC_SingleEnded(channel);
    float adc_output_volts = ads1115->computeVolts(adc_output);
    return kVoltageDividerScale * adc_output_volts / kMeasurementCurrent;
    });

if (enable_signalk_output) {
  char resistance_sk_path[80];
  snprintf(resistance_sk_path, sizeof(resistance_sk_path),
  "propulsion.%s.oilPressureResistance", sk_id.c_str());
  auto sk_output_resistance = new sensesp::SKOutputFloat(
  resistance_sk_path,
  "/Propulsion/OilPressureSensor/Resistance",
  new sensesp::SKMetadata("ohm", "Oil Pressure Resistance",
  "Raw resistance of the oil pressure sensor"));

  ConfigItem(sk_output_resistance)
  ->set_title("Oil Pressure Sensor Resistance SK Path")
  ->set_description("Signal K path for the oil pressure sensor resistance")
  ->set_sort_order(sort_order);

  resistance_sensor->connect_to(sk_output_resistance);
}

// Convert resistance to pressure (bar) using a curve
auto pressure_curve =
  (new sensesp::CurveInterpolator(nullptr, "/Propulsion/OilPressureSensor/Curve"))
  ->set_input_title("Resistance (ohm)")
  ->set_output_title("Pressure (bar)");

  ConfigItem(pressure_curve)
  ->set_title("Oil Pressure Sensor Curve")
  ->set_description("Converts resistance to oil pressure")
  ->set_sort_order(sort_order + 1);

  if (pressure_curve->get_samples().empty()) {
  pressure_curve->clear_samples();
  pressure_curve->add_sample({10.0, 0.0});
  pressure_curve->add_sample({48.0, 1.0});
  pressure_curve->add_sample({82.0, 2.0});
  pressure_curve->add_sample({116.0, 3.0});
  pressure_curve->add_sample({184.0, 5.0});
  }

resistance_sensor->connect_to(pressure_curve);

if (enable_signalk_output) {
  char pressure_sk_path[80];
  snprintf(pressure_sk_path, sizeof(pressure_sk_path),
  "propulsion.%s.oilPressure", sk_id.c_str());
  auto sk_output_pressure = new sensesp::SKOutputFloat(
  pressure_sk_path,
  "/Propulsion/OilPressureSensor/Pressure",
  new sensesp::SKMetadata("bar", "Oil Pressure", "Oil pressure in bar"));

  ConfigItem(sk_output_pressure)
  ->set_title("Oil Pressure SK Path")
  ->set_description("Signal K path for oil pressure")
  ->set_sort_order(sort_order + 2);

  pressure_curve->connect_to(sk_output_pressure);
}

return pressure_curve;
}



}  // namespace halmet