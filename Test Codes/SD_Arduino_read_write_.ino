#include <SPI.h>
#include <SD.h>

#define SD_CS 26  // Chip Select pin for SD card

void setup() {
    Serial.begin(115200);

    // Initialize SD card
    Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
        Serial.println("SD Card initialization failed!");
        return;
    }
    Serial.println("SD Card initialized successfully.");
}

void loop() {
    // Check if data is available from Serial Monitor
    if (Serial.available() > 0) {
        // Read user input
        String inputString = Serial.readStringUntil('\n');
        inputString.trim(); // Remove any accidental whitespace

        if (inputString.equalsIgnoreCase("/READ")) {
            // Read and display the file contents
            Serial.println("Reading TESTFILE.TXT...");
            File myFile = SD.open("TESTFILE.TXT");

            if (myFile) {
                while (myFile.available()) {
                    Serial.write(myFile.read());  // Print file content to Serial Monitor
                }
                myFile.close();
                Serial.println("\n--- End of File ---");
            } else {
                Serial.println("Error opening TESTFILE.TXT for reading.");
            }
        } else {
            // Write inputString to the SD card file
            File myFile = SD.open("TESTFILE.TXT", FILE_WRITE);

            if (myFile) {
                myFile.println(inputString);  // Append the input text
                myFile.close();
                Serial.println("Data written to TESTFILE.TXT");
            } else {
                Serial.println("Error opening TESTFILE.TXT for writing.");
            }
        }
    }
}
