#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#include "Arduino.h"
#include "ads_two_axis.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define ADS_RESET_PIN       (15)         // Pin number attached to ads reset line.
#define ADS_INTERRUPT_PIN   (32)         // Pin number attached to the ads data ready line. 

// function prototypes
void ads_data_callback(float * sample);
void deadzone_filter(float * sample);
void signal_filter(float * sample);
void parse_serial_port(void);

//Added functions -NR
void ads_parse_read_buffer(uint8_t * buffer, float* outSample);
uint8_t* scanI2CBus(uint8_t* deviceCount);

float ang[2];
volatile bool newData = false;

volatile bool dataReady = false; //manual interrupt setup

static uint8_t device_count = 0;
static uint8_t* addresses;
static uint8_t sensor_buffer[ADS_TRANSFER_SIZE];

//IMU Setup
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//Low-pass filter
void signal_filter(float * sample)
{
    static float filter_samples[2][6];

    for(uint8_t i=0; i<2; i++)
    {
      filter_samples[i][5] = filter_samples[i][4];
      filter_samples[i][4] = filter_samples[i][3];
      filter_samples[i][3] = (float)sample[i];
      filter_samples[i][2] = filter_samples[i][1];
      filter_samples[i][1] = filter_samples[i][0];
  
      // 20 Hz cutoff frequency @ 100 Hz Sample Rate
      filter_samples[i][0] = filter_samples[i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[i][2] + \
        0.20657208382614792f*(filter_samples[i][3] + 2*filter_samples[i][4] + filter_samples[i][5]);   

      sample[i] = filter_samples[i][0];
    }
}

void deadzone_filter(float * sample)
{
  static float prev_sample[2];
  float dead_zone = 0.5f;

  for(uint8_t i=0; i<2; i++)
  {
    if(fabs(sample[i]-prev_sample[i]) > dead_zone)
      prev_sample[i] = sample[i];
    else
      sample[i] = prev_sample[i];
  }
}
//callback funtion to read data?
void ads_data_callback(float * sample)
{
  // signal_filter(sample);
  // deadzone_filter(sample);
  
  // ang[0] = sample[0];
  // ang[1] = sample[1];
  
  // newData = true;
}
//Manual Interrupt
void IRAM_ATTR ads_interrupt_handler() { 
    dataReady = true;
}


unsigned long lastSampleTime = 0;                 //NEW LINE
const int sampleInterval = 10; // 100 Hz = every 10 ms        //NEW LINE

void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin(22, 20);
  
   //check I2C Connection
  Serial.println("Checking I2C connection...");

  addresses = scanI2CBus(&device_count);
  Wire.end();

  for (int i = 0; i < device_count; i++) {
    // Do something with each address
    Serial.print("Working with device at address 0x");
    if (addresses[i] < 16) Serial.print("0");
    Serial.println(addresses[i], HEX);

    //Add address to array
    ads_hal_update_device_addr(i, addresses[i]);
  }

  //make sure there is at least 1 device
  if(device_count < 1){
    return;
  }

  ads_init_t init;
  init.sps = ADS_100_HZ;
  init.ads_sample_callback = &ads_data_callback; //could get rid of this &?
  init.reset_pin = ADS_RESET_PIN;                 // Pin connected to ADS reset line
  init.datardy_pin = ADS_INTERRUPT_PIN;           // Pin connected to ADS data ready interrupt

  // Initialize ADS hardware abstraction layer, and set the sample rate
  ads_hal_select_device(0);
  int ret_val = ads_two_axis_init(&init);

  if(ret_val == ADS_OK)
  {
    Serial.println("Two Axis ADS initialization succeeded");
    
    pinMode(ADS_INTERRUPT_PIN, INPUT_PULLUP);  
    attachInterrupt(digitalPinToInterrupt(ADS_INTERRUPT_PIN), ads_interrupt_handler, ONLOW); 

    Serial.println("Interrupt attached successfully!");
  }
  else
  {
    Serial.print("Two Axis ADS initialization failed with reason: ");
    Serial.println(ret_val);
  }
  
  delay(100);
  
  for (int i = 0; i < device_count; i++) {
    ads_hal_select_device(i);
    ads_two_axis_enable_interrupt(true);
  }

  delay(100);

   Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(100);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;
    }

    // === Read IMU Data ===
    sensors_event_t orientationData, angVelData, linAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    // === Print All Data with Timestamp ===
    Serial.print("TIME:");
    Serial.print(currentTime);
    Serial.print(";");

 Serial.print("Bend: ");

  for (int i = 0; i < device_count; i++) {
  ads_hal_select_device(i);
  ads_hal_read_buffer(sensor_buffer, ADS_TRANSFER_SIZE);
  ads_parse_read_buffer(sensor_buffer, ang); // updates global ang[2]

  // Check if sensor data looks valid (change logic as needed)
    if (isnan(ang[0]) || isnan(ang[1]) || (ang[0] == 0.0 && ang[1] == 0.0)) {
    continue;  // Skip disconnected or invalid sensors
    }

  Serial.print(ang[0], 2);
  Serial.print(", ");
  Serial.print(ang[1], 2);

    if (i < device_count - 1) {
    Serial.print(", ");
    }
  }


    Serial.print("IMU:");
    Serial.print(orientationData.orientation.x, 2); Serial.print(","); // Roll
    Serial.print(orientationData.orientation.y, 2); Serial.print(","); // Pitch
    Serial.print(orientationData.orientation.z, 2); Serial.print(","); // Yaw
    Serial.print(linAccelData.acceleration.x, 2); Serial.print(",");   // Acc X
    Serial.print(linAccelData.acceleration.y, 2); Serial.print(",");   // Acc Y
    Serial.print(linAccelData.acceleration.z, 2); Serial.print(",");   // Acc Z
    Serial.print(angVelData.gyro.x, 2); Serial.print(",");             // Gyro X
    Serial.print(angVelData.gyro.y, 2); Serial.print(",");             // Gyro Y
    Serial.println(angVelData.gyro.z, 2);                              // Gyro Z
  
  // Handle incoming serial commands
  if (Serial.available()) {
    parse_serial_port();
  }
}


/* calibration commands for bendlab sensor */
void parse_serial_port(void)
{
    char key = Serial.read();
    
    if(key == '0')
      ads_two_axis_calibrate(ADS_CALIBRATE_FIRST, 0);
    else if(key == 'f')
      ads_two_axis_calibrate(ADS_CALIBRATE_FLAT, 90);
    else if(key == 'p')
      ads_two_axis_calibrate(ADS_CALIBRATE_PERP, 90);
    else if(key == 'c')
      ads_two_axis_calibrate(ADS_CALIBRATE_CLEAR, 0);
    else if(key == 'r')
      ads_two_axis_run(true);
    else if(key == 's')
      ads_two_axis_run(false);
    else if(key == 'f')
      ads_two_axis_set_sample_rate(ADS_200_HZ);
    else if(key == 'u')
      ads_two_axis_set_sample_rate(ADS_10_HZ);
    else if(key == 'n')
      ads_two_axis_set_sample_rate(ADS_100_HZ);
}

void ads_parse_read_buffer(uint8_t * buffer, float* outSample)
{
	if(buffer[0] == ADS_SAMPLE)
	{				
		int16_t temp = ads_int16_decode(&buffer[1]);
		outSample[0] = (float)temp/32.0f;
		
		temp = ads_int16_decode(&buffer[3]);
		outSample[1] = (float)temp/32.0f;
	}
}

uint8_t* scanI2CBus(uint8_t* deviceCount) {
  static uint8_t foundAddresses[127]; // Store detected I2C addresses
  *deviceCount = 0; // Reset count

  Serial.println("Scanning I2C bus...");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      // Print detected address
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);

      // Ignore IMU address (BNO055 is usually 0x28 or 0x29)
      if (address == 0x28 || address == 0x29) {
        Serial.println("Skipping IMU (BNO055)...");
        continue;  // Don't add it to the sensor list
      }

      // Store the valid Bendlab sensor address
      foundAddresses[*deviceCount] = address;
      (*deviceCount)++;
    }
  }

  // Print number of valid sensors found
  Serial.print("Bendlab Sensors Detected: ");
  Serial.println(*deviceCount);

  if (*deviceCount == 0) {
    Serial.println("No Bendlab sensors detected!");
  }

  return foundAddresses;
}
