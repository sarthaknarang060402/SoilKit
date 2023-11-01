#include <SoftwareSerial.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int sensor_pin = A1;                                                              // MOIS PIN = A1

#define ONE_WIRE_BUS 14   // Data wire is plugged into digital pin 14 on the Arduino    // TEMP PIN = 14 //

#define SCREEN_WIDTH 128      // OLED display width, in pixels                          // OLED
#define SCREEN_HEIGHT 64      // OLED display height, in pixels                         // OLED
#define OLED_RESET -1         // Reset pin # (or -1 if sharing Arduino reset pin)       // OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);               // OLED

#define RE 8  // RE set the RS485 module to Receiver or Transmitter mode                // RS485  RE =8
#define DE 7  // DE                                                                     // RS485  DE =7

HardwareSerial& simSerial = Serial1;  // Hardware Serial for SIM800A module             //USE 18 tx and 19 rx pins of 2560 for HW Serial    //SIM

SoftwareSerial mod(12, 13);          // Software Serial for NPK sensor (Rx, Tx)         //NPK using Software Serial

// Modbus RTU requests for reading NPK values
const byte nitro[] = {0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};                   // NPK
const byte phos[] = {0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};                    // NPK
const byte pota[] = {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};                    // NPK

OneWire oneWire(ONE_WIRE_BUS);      // Setup a oneWire instance                           // TEMP
DallasTemperature sensors(&oneWire);// Pass oneWire reference to DallasTemperature lib    // TEMP

int counter = 0;
int countDown = 5;
void setup() {
  Serial.begin(9600);    // Serial communication for debugging

  // Code for SIM800A module
  simSerial.begin(9600); // Initialize SIM800A module
  delay(6000);           // Wait for module to stabilize
  simSerial.println("AT"); // Handshake with the module
  delay(1000);
  simSerial.println("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);

  // Code for NPK sensor
  mod.begin(9600);      // Initialize NPK sensor                                              //NPK
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize OLED display                      //OLED
  delay(500);
  display.clearDisplay();
  display.setCursor(25, 15);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("  TIET");
  display.setCursor(25, 35);
  display.setTextSize(1);
  display.print("Initializing...");
  display.display();
  delay(2000);

  sensors.begin();      // Initialize DallasTemperature library for temperature sensor        //TEMP
  delay(500);
}

void loop() {
  if(counter==10)counter=6;
  byte val1 = nitrogen();
  delay(250);
  byte val2 = phosphorous();
  delay(250);
  byte val3 = potassium();
  delay(250);
  // Print values to the serial monitor
  Serial.print("Nitrogen: ");
  Serial.print(val1);
  Serial.println(" mg/kg");
  Serial.print("Phosphorous: ");
  Serial.print(val2);
  Serial.println(" mg/kg");
  Serial.print("Potassium: ");
  Serial.print(val3);
  Serial.println(" mg/kg");

  // Read moisture sensor and print
  float moisture_percentage;                                                               //MOIS start
  int sensor_analog;
  sensor_analog = analogRead(sensor_pin);
  moisture_percentage = (100 - ((sensor_analog / 1023.00) * 100));
  Serial.print("Moisture Percentage: ");
  Serial.print(moisture_percentage);
  Serial.println("%");                                                                       //MOIS end

  // Read temperature sensor and print
  sensors.requestTemperatures();  // Send the command to get temperatures                  // TEMP start
  float temperature = sensors.getTempCByIndex(0);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("C");                                                                       // TEMP end
  delay(1000);

  // Send SMS after 5 readings
  if (counter == 5) {
    sendSMS(val1, val2, val3, moisture_percentage, temperature);
  }
  if(counter>=5)
  {
    // counter = 0;
    // Display values on OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 5);
    display.print("N: ");
    display.print(val1);
    display.print(" mg/kg");
  
    display.setCursor(0, 15);
    display.print("P: ");
    display.print(val2);
    display.print(" mg/kg");
    
    display.setCursor(0, 25);
    display.print("K: ");
    display.print(val3);
    display.print(" mg/kg");

    display.setCursor(0, 35);
    display.print("Moisture: ");
    display.print(moisture_percentage);
    display.print("%");
    
    display.setCursor(0, 45);
    display.print("Temperature: ");
    display.print(temperature);
    display.print("C");

    display.display();
    delay(3000);
    // countDown=5;
  }

  if(countDown>0){`
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 5);
  display.print("calculating in ");
  display.setTextSize(2);
  display.print(countDown);
  display.display();
  delay(2000);
  }
  countDown--;
  counter++;
}
byte nitrogen() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);
  if (mod.write(nitro, sizeof(nitro)) == 8) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    byte values[7];
    for (byte i = 0; i < 7; i++) {
      values[i] = mod.read();
      Serial.print(values[i], HEX);
    }
    Serial.println();
    return values[4];
  }
  return 0;
}
byte phosphorous() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);
  if (mod.write(phos, sizeof(phos)) == 8) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    byte values[7];
    for (byte i = 0; i < 7; i++) {
      values[i] = mod.read();
      Serial.print(values[i], HEX);
    }
    Serial.println();
    return values[4];
  }
  return 0;
}
byte potassium() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);
  if (mod.write(pota, sizeof(pota)) == 8) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    byte values[7];
    for (byte i = 0; i < 7; i++) {
      values[i] = mod.read();
      Serial.print(values[i], HEX);
    }
    Serial.println();
    return values[4];
  }
  return 0;
}
void sendSMS(byte val1, byte val2, byte val3, float moisture, float temperature) {
  simSerial.println("AT+CMGS=\"+918556027636\"");  // Replace ZZ with the country code and xxxxxxxxxx with the phone number
  delay(1000);
  simSerial.print("N: ");
  simSerial.println(val1);
  simSerial.print("P: ");
  simSerial.println(val2);
  simSerial.print("K: ");
  simSerial.println(val3);
  simSerial.print("Moisture: ");
  simSerial.print(moisture);
  simSerial.println("%");
  simSerial.print("Temperature: ");
  simSerial.print(temperature);
  simSerial.println("C");
  simSerial.write(26);  // Send Ctrl+Z to indicate the end of the message
  delay(1000);
}
