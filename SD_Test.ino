// Include Libraries
#include <Wire.h>              // I2C communication
#include <Adafruit_Sensor.h>   // Adafruit sensor
#include <Adafruit_BMP3XX.h>   // BMP388 pressure/temperature sensor
#include <SD.h>                // SD card

#define SEALEVELPRESSURE_HPA (1013.25)  // Standard atmospheric pressure at sea level in hPa

// Define Objects
Adafruit_BMP3XX bmp;           // Create an object for the BMP388 sensor
File dataFile;                  // SD file object to handle writing data to SD card
unsigned long startTime;        // Start time to track elapsed time

// RGB LED pin definitions
const int redPin = 5;
const int greenPin = 4;
const int bluePin = 3;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Wait for the serial connection to be established (if connected to a computer)
  while (!Serial);

  // Print a message to the serial monitor to indicate that the setup has started
  Serial.println("Setup Starting...");

  // initialize the digitals pin as an outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Turn the LED off initially
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  // Initialize the SD card in the Teensy 4.1's built-in SD card slot
  if (!SD.begin(BUILTIN_SDCARD)) {  // Teensy 4.1 has a built-in SD card reader
    // If SD card initialization fails, print an error message and halt execution
    Serial.println("SD card initialization failed on startup!");
    digitalWrite(redPin, HIGH);  // Turn on red LED for SD card failure
    while (1);  // Infinite loop to stop the program
  }

  // Delete the existing data file if it exists
  if (SD.exists("SD_Test.csv")) {
    SD.remove("SD_Test.csv");  // Remove existing file
    Serial.println("Existing data file deleted.");
  }

  // Create a new CSV file on the SD card to store data
  dataFile = SD.open("SD_Test.csv", FILE_WRITE);  // Open (or create) a file for writing
  if (dataFile) {
    // Write the header line to the CSV file (labels for the columns)
    dataFile.println("Timestamp,Temperature (C),Pressure (hPa),Altitude (m)");
  } else {
    // If the file can't be opened, print an error message and halt execution
    Serial.println("Error opening SD_Test.csv on startup!");
    digitalWrite(redPin, HIGH);
    while (1);  // Infinite loop to stop the program
  }

  Wire.begin(); // Initialize I2C communication for BMP388

  // Initialize the BMP388 sensor with I2C communication
  if (!bmp.begin_I2C()) {  // Default I2C pins for Teensy 4.1 are SDA (pin 18) and SCL (pin 19)
    // If sensor initialization fails, print an error message and halt execution
    Serial.println("Failed to detect and initialize BMP388 on startup!");
    digitalWrite(redPin, HIGH);
    while (1);  // Infinite loop to stop the program
  }

  // Configure BMP388 sensor settings for data acquisition
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);  // Set temperature oversampling to 8x for higher precision
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);     // Set pressure oversampling to 4x
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);        // Set the IIR filter to reduce noise
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);                  // Set output data rate to 1 Hz (1 sample per second)

  startTime = millis();  // Initialize startTime when the setup is completed

  pinMode(LED_BUILTIN, OUTPUT);   // initialize digital pin LED_BUILTIN as an output.

  Serial.println("Data Collection Starting...");  // SD card initialized successfully

}

void loop() {
  // Check if 1 minute has passed
  if (millis() - startTime >= 60000) { // 1 minute in milliseconds
    dataFile.close();  // Close the file after 1 minute of data logging
    Serial.println("1 minute has passed. Stopping the data logging.");
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off (HIGH is the voltage level)
    digitalWrite(redPin, LOW);  // Turn off red LED
    digitalWrite(greenPin, LOW);  // Turn off green LED
    digitalWrite(bluePin, HIGH);   // Turn on blue LED
    while(1);  // Stop the program after data logging completes
  }

  // Verify SD is accessible and start data logging
  if (SD.mediaPresent()) {
    // Calculate the timestamp (in seconds) since the program started
    unsigned long timestamp = millis() / 1000;  // `millis()` returns milliseconds, so divide by 1000 to get seconds

    dataFile.print(timestamp);               // Write the timestamp
    dataFile.print(",");

    // Attempt to take a reading from the BMP388 sensor
    if (bmp.performReading()) {
      // If reading can be  taken, write the sensor data to the SD card in CSV format
      dataFile.print(bmp.temperature);         // Write the temperature in Celsius
      dataFile.print(",");                     // CSV delimiter
      dataFile.print(bmp.pressure / 100.0);    // Write the pressure in hPa (Pa to hPa conversion)
      dataFile.print(",");                     // CSV delimiter
      dataFile.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));  // Write the calculated altitude in meters   
      digitalWrite(redPin, LOW);    // Turn off red LED
      digitalWrite(greenPin, HIGH);   // Turn on green LED for successful logging
      dataFile.flush();
    } else {
      // If reading fails, print an error message and halt the program
      Serial.println("Failed to read BMP Sensor!");
      digitalWrite(greenPin, LOW); // Ensure green LED is off
      digitalWrite(redPin, HIGH); // Make sure red LED is on
      while(1);
    }

    // Log data to the file if the SD file can be found
    if (SD.exists("SD_Test.csv")) {
      // If reading can be  taken, write the sensor data to the SD card in CSV format
      dataFile.print(bmp.temperature);         // Write the temperature in Celsius
      dataFile.print(",");                     // CSV delimiter
      dataFile.print(bmp.pressure / 100.0);    // Write the pressure in hPa (Pa to hPa conversion)
      dataFile.print(",");                     // CSV delimiter
      dataFile.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));  // Write the calculated altitude in meters   
      digitalWrite(redPin, LOW);    // Turn off red LED
      digitalWrite(greenPin, HIGH);   // Turn on green LED for successful logging
      dataFile.flush();

    } else {
      // If reading fails, print an error message and halt the program
      Serial.println("File cannot be found");
      digitalWrite(greenPin, LOW); // Ensure green LED is off
      digitalWrite(redPin, HIGH); // Make sure red LED is on
      while(1);

    }



  } else { 
    Serial.println("Error writing to SD_Test.csv, SD card may be removed");
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, HIGH);
    // Start retry attempts
    int retryCount = 0; // Create integer to track retries 
    const int maxRetries = 10; // Set max retry number
    // Enter while loop for up to 10 retries
    while (retryCount < maxRetries) {
      // Try to reconnect to the SD card 
      if (SD.begin(BUILTIN_SDCARD)) {
        break; // exits the while loop if successful
      } else {
        // Print to serial if reconnect attempt unsuccessful
        Serial.print("Retrying SD card initialization... Attempt ");
        Serial.println(retryCount + 1); 
        retryCount++; //increase count by 1
        delay(500); // wait 0.5 seconds before trying again
      }
    }
    // Stop code once max retry attempts reach
    if (retryCount == maxRetries) {
      Serial.print("SD is lost...");
      while(1);
    }
    Serial.println("SD connection recovered. Data collection resuming..."); // Signify SD connection recovered
  }

  delay(1000);
} 