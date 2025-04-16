#include "Arduino.h"
#include "ads_two_axis.h"
#include "Wire.h"

// Configuration
#define ADS_RESET_PIN       15
#define ADS_INTERRUPT_PIN   32
#define I2C_SDA             22
#define I2C_SCL             20

#define TARGET_SENSOR_ADDR  0x13  // <--- Set the specific sensor address you want

float ang[2];
uint8_t sensor_buffer[ADS_TRANSFER_SIZE];
bool target_found = false;

// === Choose which parsing to use ===

// [1] ANGLES (°) — DEFAULT
void ads_parse_read_buffer(uint8_t * buffer, float* outSample)
{
  if(buffer[0] == ADS_SAMPLE)
  {				
    int16_t temp = ads_int16_decode(&buffer[1]);
    outSample[0] = (float)temp / 32.0f;
    
    temp = ads_int16_decode(&buffer[3]);
    outSample[1] = (float)temp / 32.0f;
  }
}

/*
// [2] VOLTAGE (V) — Uncomment to use
void ads_parse_read_buffer(uint8_t * buffer, float* outSample)
{
  if(buffer[0] == ADS_SAMPLE)
  {				
    int16_t rawX = ads_int16_decode(&buffer[1]);
    int16_t rawY = ads_int16_decode(&buffer[3]);

    outSample[0] = (float)rawX * (3.3f / 65536.0f);
    outSample[1] = (float)rawY * (3.3f / 65536.0f);
  }
}
*/

// Scan I2C bus for the target sensor address
bool findTargetSensor() {
  Serial.println("Scanning I2C bus for target sensor...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0 && address == TARGET_SENSOR_ADDR) {
      Serial.print("Target sensor found at address 0x");
      Serial.println(address, HEX);
      return true;
    }
  }
  Serial.println("Target sensor not found.");
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin(I2C_SDA, I2C_SCL);

  target_found = findTargetSensor();
  if (!target_found) return;

  // Tell the HAL what address to use for device index 0
  ads_hal_update_device_addr(0, TARGET_SENSOR_ADDR);

  // Initialize the target sensor only
  ads_init_t init;
  init.sps = ADS_100_HZ;
  init.ads_sample_callback = nullptr;
  init.reset_pin = ADS_RESET_PIN;
  init.datardy_pin = ADS_INTERRUPT_PIN;

  ads_hal_select_device(0);
  int result = ads_two_axis_init(&init);
  if (result != ADS_OK) {
    Serial.println("Sensor failed to initialize.");
    target_found = false;
  }

  delay(100);
}

void loop() {
  if (!target_found) return;

  ads_hal_select_device(0);
  ads_hal_read_buffer(sensor_buffer, ADS_TRANSFER_SIZE);
  ads_parse_read_buffer(sensor_buffer, ang);

  Serial.print("Sensor 0 (0x");
  Serial.print(TARGET_SENSOR_ADDR, HEX);
  Serial.print("): ");
  Serial.print(ang[0]);
  Serial.print(", ");
  Serial.println(ang[1]);

  delay(100); // Adjust for desired sampling rate
}
