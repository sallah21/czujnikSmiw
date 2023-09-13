#include <Arduino.h>
#include "Matter.h"
#include <app/server/OnboardingCodesUtil.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
using namespace chip;
using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::endpoint;

SensirionI2CScd4x scd4x;


// Cluster and attribute ID used by Matter light device
const uint32_t CLUSTER_ID = TemperatureMeasurement::Id; //OnOff::Id;
const uint32_t ATTRIBUTE_ID =TemperatureMeasurement::Attributes::MeasuredValue::Id;///OnOff::Attributes::OnOff::Id; 

const uint32_t CLUSTER_ID2 = RelativeHumidityMeasurement::Id; //OnOff::Id;
const uint32_t ATTRIBUTE_ID2 = RelativeHumidityMeasurement::Attributes::MeasuredValue::Id;

const uint32_t CLUSTER_ID3 = PressureMeasurement::Id; //OnOff::Id;
const uint32_t ATTRIBUTE_ID3 =PressureMeasurement::Attributes::MeasuredValue::Id;

uint16_t temperature_endpoint_id = 1;
uint16_t humidity_endpoint_id = 2;
uint16_t co2_endpoint_id = 3;


attribute_t *attribute_ref;
attribute_t *attribute_ref2;
attribute_t *attribute_ref3;

 static void on_device_event(const ChipDeviceEvent *event, intptr_t arg) {}


static esp_err_t on_identification(identification::callback_type_t type, uint16_t endpoint_id,
                                   uint8_t effect_id, uint8_t effect_variant, void *priv_data) {
  return ESP_OK;
}

// Listener on attribute update requests.
// In this example, when update is requested, path (endpoint, cluster and attribute) is checked
// if it matches light attribute. If yes, LED changes state to new one.
static esp_err_t on_attribute_update(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                     uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data) {
  Serial.print("matter updated");
  return ESP_OK;
}

void printUint16Hex(uint16_t value) {
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}

uint16_t co2 = 0;
float temperature = 0.0f;
float humidity = 0.0f;


void setup() {
  // put your setup code here, to run once:


  // Enable debug logging
  esp_log_level_set("*", ESP_LOG_DEBUG);

  // Setup Matter node
  node::config_t node_config;
  node_t *node = node::create(&node_config, on_attribute_update, on_identification);

  // // Setup Light endpoint / cluster / attributes with default values
  //on_off_light::config_t light_config;
    //light_config.on_off.on_off = false;
  //light_config.on_off.lighting.start_up_on_off = false;

  //TEMPERATURE
  temperature_sensor::config_t temp_config;
  temp_config.temperature_measurement.measured_value = temperature ;
  endpoint_t *endpoint = temperature_sensor::create(node, &temp_config, ENDPOINT_FLAG_NONE, NULL);


  //HUMIDITY
  humidity_sensor::config_t hum_config;
  hum_config.relative_humidity_measurement.measured_value = humidity;
  endpoint_t *endpoint2 = humidity_sensor::create(node, &hum_config, ENDPOINT_FLAG_NONE, NULL);


  //CO2
  pressure_sensor::config_t co2_config;
  co2_config.pressure_measurement.pressure_measured_value = co2;
  endpoint_t *endpoint3 = pressure_sensor::create(node, &co2_config, ENDPOINT_FLAG_NONE, NULL);

  // Save on/off attribute reference. It will be used to read attribute value later.
  attribute_ref = attribute::get(cluster::get(endpoint, CLUSTER_ID), ATTRIBUTE_ID);
  attribute_ref2 = attribute::get(cluster::get(endpoint2, CLUSTER_ID2), ATTRIBUTE_ID2);
  attribute_ref3 = attribute::get(cluster::get(endpoint3, CLUSTER_ID3), ATTRIBUTE_ID3);

  // Save generated endpoint id
  temperature_endpoint_id = endpoint::get_id(endpoint);
  humidity_endpoint_id = endpoint::get_id(endpoint2);
  co2_endpoint_id = endpoint::get_id(endpoint3);
  // Setup DAC (this is good place to also set custom commission data, passcodes etc.)
  //esp_matter::set_custom_dac_provider(chip::Credentials::Examples::GetExampleDACProvider());

  // Start Matter device
  esp_matter::start(on_device_event);




  Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    Wire.begin();

    uint16_t error;
    char errorMessage[256];

    scd4x.begin(Wire);

    // stop potentially previously started measurement
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    error = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        printSerialNumber(serial0, serial1, serial2);
    }

    // Start Measurement
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    Serial.println("Waiting for first measurement... (5 sec)");

}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t error;
    char errorMessage[256];

    delay(100);

    // Read Measurement

    bool isDataReady = false;
    error = scd4x.getDataReadyFlag(isDataReady);
    if (error) {
        Serial.print("Error trying to execute getDataReadyFlag(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
        return;
    }
    if (!isDataReady) {
        return;
    }
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error) {
        Serial.print("Error trying to execute readMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else if (co2 == 0) {
        Serial.println("Invalid sample detected, skipping.");
    } else {
        Serial.print("Co2:");
        Serial.print(co2);
        Serial.print("\t");
        Serial.print("Temperature:");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity);
    }
}

