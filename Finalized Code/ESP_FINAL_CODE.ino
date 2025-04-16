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
  #include <Adafruit_NeoPixel.h>
  #include <SPI.h>
  #include <SD.h>

  #define ADS_RESET_PIN       (15)         // Pin number attached to ads reset line.
  #define ADS_INTERRUPT_PIN   (32)         // Pin number attached to the ads data ready line. 

  #define PIN        0    // Pin the NeoPixel is connected to
  #define NUMPIXELS  1     // Only one built-in NeoPixel
  #define BUTTON_PIN  38 //Input Button 

  #define SD_CS 26
  File dataFile;
  String filename;

  Adafruit_NeoPixel pixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

  bool headerWritten = false;
  uint32_t readyBlinkColor = 0; // Will hold blue or yellow depending on sensor check

  //NeoPixelSetup
 
  // States
  enum State {
    IDLE,           // Before setup complete
    READY,          // Setup complete, waiting for button press
    COLLECTING      // Data collection running
  };

  State systemState = IDLE;
  bool buttonPressed = false;
  unsigned long lastBlinkTime = 0;
  bool ledState = false;

void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

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

  String createNewFilename() {
    int index = 1;
    String filename;

    while (true) {
      filename = "/data_" + String(index) + ".csv";
      if (!SD.exists(filename)) {
      return filename;
      }
    index++;
    if (index > 9999) return "/data_overflow.csv";  // failsafe
    }
  }

void setup() {
  Serial.begin(115200);

  // === UI Setup ===
  pixel.begin();
  pixel.setBrightness(50);
  setLEDColor(0, 0, 255);  // Solid blue = power on
  pinMode(BUTTON_PIN, INPUT);
  Wire.begin(22, 20);

  // === Initialization Flags ===
  bool imuInitialized = false;
  bool sdInitialized = false;
  bool bendlabSensorsInitialized = false;

  // === SD Card Setup ===
  if (!SD.begin(SD_CS)) {
    Serial.println("Initialization failed: SD card not detected.");
    setLEDColor(255, 0, 0); // Solid red
    while (true); // Halt
  } else {
    sdInitialized = true;
    Serial.println("SD initialized.");
  }

  filename = createNewFilename();
  Serial.print("Logging to: ");
  Serial.println(filename);

  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Initialization failed: Could not create log file.");
    setLEDColor(255, 0, 0); // Solid red
    while (true); // Halt
  }

  if (dataFile.size() == 0) {
    dataFile.println("Timestamp (ms),Close Hip X,Close Hip Y,Close Knee X,Close Knee Y,Far Hip X,Far Hip Y,Far Knee X,Far Knee Y,IMU Orientation X,IMU Orientation Y,IMU Orientation Z,Accel X,Accel Y,Accel Z,Gyro X,Gyro Y,Gyro Z");
    dataFile.flush();
    Serial.println("Header printed.");
  } else {
    Serial.println("Header already exists.");
  }

  // === I2C Device Scan ===
  Serial.println("Checking I2C connection...");
  addresses = scanI2CBus(&device_count);
  Wire.end();

  for (int i = 0; i < device_count; i++) {
    Serial.print("Found I2C device at 0x");
    if (addresses[i] < 16) Serial.print("0");
    Serial.println(addresses[i], HEX);
    ads_hal_update_device_addr(i, addresses[i]);
  }

  if (device_count < 1) {
    Serial.println("No BendLab devices found.");
    setLEDColor(255, 0, 0); // Red blink
    while (true) {
      setLEDColor(255, 0, 0);
      delay(500);
      setLEDColor(0, 0, 0);
      delay(500);
    }
  }

  // === Bendlab Sensor Setup ===
  ads_init_t init;
  init.sps = ADS_100_HZ;
  init.ads_sample_callback = &ads_data_callback;
  init.reset_pin = ADS_RESET_PIN;
  init.datardy_pin = ADS_INTERRUPT_PIN;

  ads_hal_select_device(0);
  int ret_val = ads_two_axis_init(&init);

  if (ret_val == ADS_OK) {
    //Serial.println("Two Axis ADS initialization succeeded.");
    pinMode(ADS_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ADS_INTERRUPT_PIN), ads_interrupt_handler, ONLOW);
    //Serial.println("Interrupt attached successfully!");
    Serial.println("Initialization failed: BendLab sensors not detected");
    bendlabSensorsInitialized = true;
  } else {
    Serial.print("ADS init failed with reason: ");
    Serial.println(ret_val);
  }

  for (int i = 0; i < device_count; i++) {
    ads_hal_select_device(i);
    ads_two_axis_enable_interrupt(true);
  }

  // === IMU Setup ===
  Serial.println("Initializing IMU...");
  if (bno.begin()) {
    Serial.println("IMU initialized.");
    imuInitialized = true;
  } else {
    Serial.println("Initialization failed: IMU not detected.");
  }

  if (device_count < 4 || !imuInitialized) {
    Serial.println("Warning: Less than 4 BendLab sensors or no IMU detected.");
    readyBlinkColor = pixel.Color(200, 150, 0); // Yellow
  } else {
    readyBlinkColor = pixel.Color(0, 0, 255);   // Blue
  }
  Serial.println("System READY.");
  systemState = READY;
  lastBlinkTime = millis();
}


void loop() {
  unsigned long currentTime = millis();

  switch (systemState) {

    case READY: {
      // Blink yellow or blue based on sensor init
      if (currentTime - lastBlinkTime >= 500) {
        ledState = !ledState;

        if (ledState){
          pixel.setPixelColor(0, readyBlinkColor);} // Yellow or Blue
        else{
          pixel.setPixelColor(0, readyBlinkColor);}            // Off

        pixel.show();
        lastBlinkTime = currentTime;
      }

      // Detect button press
      static bool lastButtonState = HIGH;
      bool currentButtonState = digitalRead(BUTTON_PIN);

      if (lastButtonState == HIGH && currentButtonState == LOW) {
        buttonPressed = true;
      }
      lastButtonState = currentButtonState;

      if (buttonPressed) {
        Serial.println("Reinitializing IMU before data collection...");

        if (bno.begin()) {
          Serial.println("IMU reinitialized successfully.");
        } else {
          Serial.println("IMU reinitialization failed.");
        }

        systemState = COLLECTING;
        buttonPressed = false;
        lastBlinkTime = currentTime;
        ledState = true;
      }
      break;
    }
    case COLLECTING: {
      // Blink green: 1s on, 4s off
      if ((ledState && currentTime - lastBlinkTime >= 1000) ||
          (!ledState && currentTime - lastBlinkTime >= 4000)) {
        ledState = !ledState;

        if (ledState)
          setLEDColor(0, 255, 0); // Green ON
        else

          setLEDColor(0, 0, 0);   // OFF

        lastBlinkTime = currentTime;
      }

      // Sample sensors at interval
      if (currentTime - lastSampleTime >= sampleInterval) {
        lastSampleTime = currentTime;

        // === Read IMU Data ===
        sensors_event_t orientationData, angVelData, linAccelData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno.getEvent(&linAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

        // === Build Output String ===
        String output = String(currentTime) + ",";

        // === Read Bend Sensor Data ===
        for (int i = 0; i < device_count; i++) {
          ads_hal_select_device(i);
          ads_hal_read_buffer(sensor_buffer, ADS_TRANSFER_SIZE);
          ads_parse_read_buffer(sensor_buffer, ang); // updates global ang[2]

          if (isnan(ang[0]) || isnan(ang[1]) || (ang[0] == 0.0 && ang[1] == 0.0)) {
            continue;
          }

          output += String(ang[0], 2) + "," + String(ang[1], 2) + ",";
        }

        // === Append IMU Data ===
        output += String(orientationData.orientation.x, 2) + ",";
        output += String(orientationData.orientation.y, 2) + ",";
        output += String(orientationData.orientation.z, 2) + ",";
        output += String(linAccelData.acceleration.x, 2) + ",";
        output += String(linAccelData.acceleration.y, 2) + ",";
        output += String(linAccelData.acceleration.z, 2) + ",";
        output += String(angVelData.gyro.x, 2) + ",";
        output += String(angVelData.gyro.y, 2) + ",";
        output += String(angVelData.gyro.z, 2); // last entry, no comma

        // === Print and Log ===
        Serial.println(output);

        if (dataFile) {
          dataFile.println(output);
          dataFile.flush();
        }
      }

      // === Handle Serial Commands ===
      if (Serial.available()) {
        parse_serial_port();
      }
      break;
    }
    case IDLE:{
    default:
      break;
    }
  }
}

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


