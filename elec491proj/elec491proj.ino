/*
 * --- HiFly obstacle avoidance sensor stack for Tello ---
 * Description:
 * This program utlizies 2 OPT3101 sensors and one VL53L0X ToF sensor.
 * All the sensor readings are sent through wifi to a static IP and port.
 * An LED matrix (8 LEDs) is used to provide direct feedback on the sensor readings.
 * The LED matrix is also used to implement user-feedback functionality such as a green blink.
 * 
 * Author: Benedek Hegedus, Huawei Canada
 */

// ---- DEBUG START ----
#define LED_PWR 13
int led_1_blink_time = 200;
long led_1_last_blink;
// ---- DEBUG END ----


// ------ WIFI START ------
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>



// ------###### NETWORK CONFIG START ######------

#define SECRET_SSID "TP-Link_hifly"
#define SECRET_PASS ""
 
IPAddress local_ip(192, 168, 0, 152); // for static ip   

#define ATLASDK_IP "192.168.0.101"
const int16_t ATLASDK_PORT = 9121;

// ------###### NETWORK CONFIG END ######------



int status = WL_IDLE_STATUS;
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[8]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

bool send_flag = false;

WiFiUDP Udp;
// ------ WIFI END ------



// ------ VL53L0X ToF SENSOR START ------
#include "Arduino.h"
#include "DFRobot_VL53L0X.h"
DFRobot_VL53L0X tofSensor;
// ------ VL53L0X ToF SENSOR END ------



// ------ OPT3101 SENSOR START ------
#include <OPT3101.h>
#include <Wire.h>

const uint8_t TRANSISTOR_PIN_1 = 3;
const uint8_t TRANSISTOR_PIN_2 = 4;

const int16_t MAX_DISTANCE = 1000;
const int16_t MIN_DISTANCE = 1;

OPT3101 sensor1;
OPT3101 sensor2;
uint16_t amplitudes[6];
int16_t distances[7];
// ------ OPT3101 SENSOR END ------



// ------ 8 LED RGB START ------

#include <Adafruit_NeoPixel.h>
#ifdef _AVR_
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        2 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 8 // Popular NeoPixel ring size

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

uint8_t RGB_values[3];

// ------ 8 LED RGB END ------

    
// ------ artificial potential field BEGIN ------

  int NUM_SENSORS = 7;
  double K_REPEL = 6000;
  double K_ATTRACT = 0.05;
  double RADIUS_MAX = 500;         // RADIUS_MAX radius, beyond this distance, no repel force
  
// ------ artificial potential field END ------

void setup()
{ 
 // DEBUG
  pinMode(LED_PWR, OUTPUT);

  // Init pins to switch between OPT sensors
  pinMode(TRANSISTOR_PIN_1, OUTPUT);
  pinMode(TRANSISTOR_PIN_2, OUTPUT);

  Wire.begin();
  
  Serial.println("Init Sensor 1 start");

  initLEDs(64); // init LEDs and set brightness to 64 (out of 255)

  setLEDs(1);

  initSensor1();

  initSensor2();

  setLEDs(2);

  init_tof_sensor();

  setLEDs(3);

  // initWIFI();

  digitalWrite(LED_PWR, HIGH);

}

void loop()
{
  readOPTs();  
  readTofSensor();
  setLEDs();
  sendSensorData();
  handleRequest();


  // Blink built in LED to easly debug if Arduino crashes
  if((millis() - led_1_last_blink) >= led_1_blink_time){
    if(digitalRead(13) == HIGH){
      digitalWrite(13, LOW);
    }else{
      digitalWrite(13, HIGH);
    }
    led_1_last_blink = millis();
  }


  // Print sensor readings through serial - can be commented out 
  for (uint8_t i = 0; i < 7; i++)
  {
    Serial.print(distances[i]);
    Serial.print(", ");
  }
  
  Serial.println();

  obs_avoid_APF(distances);
}

// ------ artificial potential field algo START ----- //
void obs_avoid_APF(int16_t distances[]){
  int dest_dist = distances[6]; // inputs from alpha pose: distance
  //the distances go in a clockwise order
  //the positive x axis is 0 degrees. 
  //so, the corresponding angles are {-35, -90, -125, 125, 90, 35, 0} degrees. 
  //obs_angle is in radians
  double obs_angle[NUM_SENSORS] = {-0.610865, -1.5708, -2.18166, 2.18166, 1.5708, 0.610865, 0};

 //convert this to x's and y's
  double obs_x[NUM_SENSORS];
  double obs_y[NUM_SENSORS];
  
  Serial.print("obstacle locations");
  for (int i = 0; i < NUM_SENSORS; i++){
    obs_x[i] = get_x(obs_angle[i], distances[i]);
    obs_y[i] = get_y(obs_angle[i], distances[i]);
    
    Serial.print(" obs_x: ");
    Serial.print(obs_x[i]);
    Serial.print(" obs_y: ");
    Serial.print(obs_y[i]);
  }
  Serial.println();

  //calculate the attractive force
  double fx_attract = K_ATTRACT * (dest_dist);
  double fy_attract = K_ATTRACT * (dest_dist);

  double fx_repels[NUM_SENSORS];
  double fy_repels[NUM_SENSORS];

    Serial.print("repel forces:  ");
  //calculate the repel force
  for (int i = 0; i < NUM_SENSORS; i++){
    fx_repels[i] = repel_x(0, obs_x[i], 0, obs_y[i], dest_dist);
    fy_repels[i] = repel_y(0, obs_x[i], 0, obs_y[i], dest_dist);
    Serial.print(" x: ");
    Serial.print(fx_repels[i]);
    Serial.print(" y: ");
    Serial.print(fx_repels[i]);
  }
    Serial.println();

  //calculate net force  
  double fx_net = sum_func(fx_repels, NUM_SENSORS) + fx_attract;
  double fy_net = sum_func(fy_repels, NUM_SENSORS) + fy_attract;

  Serial.print("Fx net:");
  Serial.print(fx_net);
  Serial.print("Fy net:");
  Serial.print(fy_net);
  Serial.println();

}

double sum_func(double arr[], int size){
  double sum = 0;
  for (int i = 0; i < size; i++){
    sum+=arr[i];    
  }
  return sum;
}

double get_x(double angle, int distance) { //in radians
  double x_val = distance * cos (angle);
  if (x_val < 1.0) return 1.0;
  else return x_val;
}

double get_y(double angle, int distance) {
  double y_val = distance * sin (angle);
  if (y_val < 1.0) return 1.0;
  else return y_val;
}


// x x value of the drone in global coordinates
// xob an array of the x values of the obstacles
// y y value of the drone in global coordinates
// yob an array of the y values of the obstacles
// distance the distance to the destination

double repel_x(double x, double xob, double y, double yob, double distance){
  double fx_repel = 0;
  if (distance <= RADIUS_MAX) {
    fx_repel = 0.5 * K_REPEL * ((1 / distance) - (1 / RADIUS_MAX)) * (0.5 * pow(((xob - x) * (xob - x) + (yob - y) * (yob - y)), -0.5) * (-2 * (xob - x)) / distance);
  } else {
    fx_repel = 0;
  }
  return fx_repel;
}

double repel_y(double x, double xob, double y, double yob, double distance){
  double fy_repel = 0;
  if (distance <= RADIUS_MAX) {
     fy_repel = 0.5 * K_REPEL * ((1 / distance) - (1 / RADIUS_MAX)) * (0.5 * pow(((xob - x) * (xob - x) + (yob - y) * (yob - y)), -0.5) * (-2 * (yob - y)) / distance);
  } else {
    fy_repel = 0;
  }
  return fy_repel;
}

// ------ artificial potential field algo END ----- //

// ------ OPT3101 SENSOR FUNCTIONS START ------
void readOPTs(){
  startSensor1();
  startSensor2();
  finSensor1(0);
  startSensor1();
  finSensor2(0);
  startSensor2();
  finSensor1(1);
  startSensor1();
  finSensor2(1);
  startSensor2();
  finSensor1(2);
  finSensor2(2);
}

void startSensor1(){
  setSensor1();
  sensor1.startSample();
  return;
}

void startSensor2(){
  setSensor2();
  sensor2.startSample();
  return;
}

void finSensor1(uint8_t channel){
  setSensor1();
  int timeout_count = 0;
  while(!sensor1.isSampleDone() && timeout_count < 100000){
    timeout_count++;
  }
  if(timeout_count >= 100000){
    Serial.println("TIMEOUT!!!");
  }
  
  sensor1.readOutputRegs();
  distances[channel] = sensor1.distanceMillimeters;
  sensor1.nextChannel();
  return;
}

void finSensor2(uint8_t channel){
  setSensor2();
  int timeout_count = 0;
  while(!sensor2.isSampleDone() && timeout_count < 100000){
    timeout_count++;
  }
  if(timeout_count >= 100000){
    Serial.println("TIMEOUT!!!");
  }
  
  sensor2.readOutputRegs();
  distances[channel+3] =  sensor2.distanceMillimeters;
  sensor2.nextChannel();
  return;
}


void initSensor1(){
  setSensor1();

  sensor1.init();
  if (sensor1.getLastError())
  {
    //Serial.print(F("Failed to initialize OPT3101 1: error "));
    //Serial.println(sensor1.getLastError());
    while (1) {}
  }

  sensor1.setFrameTiming(128);
  sensor1.setChannel(0);
  sensor1.setBrightness(OPT3101Brightness::High);
  return;
}


void initSensor2(){
  setSensor2();

  sensor2.init();
  if (sensor2.getLastError())
  {
    //Serial.print(F("Failed to initialize OPT3101 2: error "));
    //Serial.println(sensor2.getLastError());
    while (1) {}
  }

  sensor2.setFrameTiming(128);
  sensor2.setChannel(0);
  sensor2.setBrightness(OPT3101Brightness::High);
  return;
}



void setSensor1(){
  delayMicroseconds(10);
  digitalWrite(TRANSISTOR_PIN_2, LOW);
  digitalWrite(TRANSISTOR_PIN_1, HIGH);
  delayMicroseconds(10);
  return;
}

void setSensor2(){
  delayMicroseconds(10);
  digitalWrite(TRANSISTOR_PIN_1, LOW);
  digitalWrite(TRANSISTOR_PIN_2, HIGH);
  delayMicroseconds(10);
  //setSensor1();
  return;
}
// ------ OPT3101 SENSOR FUNCTIONS END ------



// ------ RGB LED FUNCTIONS START ------

void setLEDs (){ 
  distanceToRGB(distances[0]);
  pixels.setPixelColor(1, pixels.Color(RGB_values[0], RGB_values[1], RGB_values[1]));

  distanceToRGB(distances[1]);
  pixels.setPixelColor(2, pixels.Color(RGB_values[0], RGB_values[1], RGB_values[1]));

  distanceToRGB(distances[2]);
  pixels.setPixelColor(3, pixels.Color(RGB_values[0], RGB_values[1], RGB_values[1]));

  distanceToRGB(distances[3]);
  pixels.setPixelColor(4, pixels.Color(RGB_values[0], RGB_values[1], RGB_values[1]));

  distanceToRGB(distances[4]);
  pixels.setPixelColor(5, pixels.Color(RGB_values[0], RGB_values[1], RGB_values[1]));

  distanceToRGB(distances[5]);
  pixels.setPixelColor(6, pixels.Color(RGB_values[0], RGB_values[1], RGB_values[1]));  
  

  distanceToRGB(distances[6]);
  pixels.setPixelColor(0, pixels.Color(RGB_values[0], RGB_values[1], RGB_values[1]));
  
  pixels.show();   // Send the updated pixel colors to the hardware.
  return;
}

// Uses RGB_values global array
void distanceToRGB(int16_t distance){
  int16_t bounded_distance = distance;
  
  if(distance >= MAX_DISTANCE){
    bounded_distance = MAX_DISTANCE;
  }
  if(distance < MIN_DISTANCE){
    bounded_distance = MIN_DISTANCE;
  }

  float distance_percent = (float)(bounded_distance - MIN_DISTANCE) / (float)(MAX_DISTANCE - MIN_DISTANCE);
 
  RGB_values[0] = (int8_t)((1.0 - distance_percent) * 10.0); // scale RED inverse with distance
  RGB_values[1] = (int8_t)(distance_percent * 10.0); // scale GREEN directly with distance
  RGB_values[2] = 0;

  return;
}

void initLEDs(uint8_t brightness){
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
  
  #if defined(_AVR_ATtiny85_) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif
  // END of Trinket-specific code.

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(brightness);
  pixels.clear(); // Set all pixel colors to 'off'
  return;
}


void setLEDs(uint8_t color){
  // set LED green
  pixels.clear(); // Set all pixel colors to 'off'
  for(int i=0; i<7; i++) { 

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    if (color == 0){
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }
    if (color == 1){
    pixels.setPixelColor(i, pixels.Color(50, 0, 0));
    }
    if (color == 2) {
    pixels.setPixelColor(i, pixels.Color(0, 50, 0));
    }
    if (color == 3) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 50));
    }
    pixels.show();   // Send the updated pixel colors to the hardware.

  }
  return;
}
// ------ RGB LED FUNCTIONS END ------



// ------ ToF SENSOR FUNCTIONS START ------
void init_tof_sensor(){
  // default address is 0x50
  tofSensor.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  tofSensor.setMode(tofSensor.eContinuous,tofSensor.eHigh);
  //Laser rangefinder begins to work
  tofSensor.start();
  return;
}

void readTofSensor(){
  distances[6] = tofSensor.getDistance();
  return;
}
// ------ ToF SENSOR FUNCTIONS END ------



// ------ WIFI FUNCTIONS START ------
void initWIFI(){
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

 
  // attempt to connect to WiFi network:
  WiFi.config(local_ip);  // use static ip
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(3000);
  }
  //Serial.println("Connected to WiFi");
  printWifiStatus();

  //Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);

  return;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  return;
}

void sendSensorData(){

   int16_t bounded_distances[7];

   for(int i=0; i < 7; i++){
    if (distances[i] <= MIN_DISTANCE){
      bounded_distances[i] = MIN_DISTANCE;
    }
    else if (distances[i] > MAX_DISTANCE){
      bounded_distances[i] = MAX_DISTANCE;
    }
    else {
      bounded_distances[i] = distances[i];
    }
   }
  
   String semicolon = String(";");
   String sensorData = (String(bounded_distances[3]) + semicolon
                        + String(bounded_distances[4]) + semicolon  
                        + String(bounded_distances[5]) + semicolon  
                        + String(bounded_distances[0]) + semicolon  
                        + String(bounded_distances[1]) + semicolon  
                        + String(bounded_distances[2]) + semicolon 
                        + String(bounded_distances[6]));
                        
   Udp.beginPacket(ATLASDK_IP, ATLASDK_PORT);
   Udp.write(sensorData.c_str());
   //Serial.println(sensorData);
   Udp.endPacket();
   return;
}


// Run a function based on value received 
void handleRequest(){
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  
  if (packetSize) {
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 7);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    if(packetBuffer[0] == '1'){
      userFeedback1();
    }
    if(packetBuffer[0] == '2'){
      userFeedback2();
    }
    
    // Not used by default, but can use for condition start
    // sending sensor data.
    if(packetBuffer[0] == '5'){
      startSending();
    }    
  }
  return;
}

void userFeedback1(){
  // Blink LED GREEN
  pixels.clear(); // Set all pixel colors to 'off'
  for(int i=0; i<7; i++) { 

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0, 100, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.

  }
  delay(200); // Delay so that user can notice blink
  return;
}

void userFeedback2(){
  // Blink LED BLUE
  pixels.clear(); // Set all pixel colors to 'off'
  for(int i=0; i<7; i++) { 

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0, 0, 100));
    pixels.show();   // Send the updated pixel colors to the hardware.

  }
  delay(1000); // Delay so that user can notice blink
  return;
}

// By default, always sending sensor data,
// so this is not needed.
void startSending(){
  send_flag = true;
  return;
}