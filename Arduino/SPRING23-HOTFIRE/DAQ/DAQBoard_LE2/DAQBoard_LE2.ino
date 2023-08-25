/*
This code runs on the DAQ ESP32 and has a couple of main tasks.
1. Read sensor data
2. Send sensor data to COM ESP32
3. Actuate hotfire sequence
*/

/*
Changes:
Removes duplicate codes from functions.
Merge state and currDAQState to DAQState.
Rename commandedState to COMState.
Rename getData() to fetchCOMState().
Rename CheckDebug() to syncDAQState().
Move duplicated code in loop()'s switch case statement to before the switch.
Remove a lot of CheckDebug() call because commandedState is only updated at 2 locations.
Changed idle to setAllBitsUp() instead of setting each mosfet individually.
Fixed abort_sequence setBitDown() instead of setBitUp() for the second vent lox if statement.
Rename pcf to mosfet_pcf.
Change order of functions to have better grouping and flow.
Change concat to += for printSensorReading().
Add if statement to not call mosfet_pcf if it is not found. (caused slowness if mosfet board is not plugged in).
  Abstract pcf.setLeftBitUp with mosfetCloseValve(mosfet_num).
Renamed the wifi data send functions to have better flow.

TODO:
Need to ask Liam what should happen in each state and update it in general.
Check threshold in press(). The variable is used in some comparison, but not other.
Why is there a while loop in press()? Isn't press() already called in loop()? The point may be to prevent moving to QD before press finish. In this case, we can  just assert that ethComplete and loxComplete is true before changing state.
Remove redundant call to pcf.
*/

//::::::Libraries::::::://
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include "HX711.h"
#include "Adafruit_MAX31855.h"
#include <EasyPCF8575.h>


//::::::Global Variables::::::://

// DEBUG TRIGGER: SET TO 1 FOR DEBUG MODE.
int DEBUG = 0;     // Simulate LOX and Eth fill.
int WIFIDEBUG = 0; // Don't send data.

// MODEL DEFINED PARAMETERS FOR TEST/HOTFIRE. Pressures in psi //
float pressureFuel  = 412;  // Set pressure for fuel: 412
float pressureOx    = 445;  // Set pressure for lox: 445
float threshold     = 0.97; // re-pressurrization threshold (/1x)
float ventTo        = 10;   // c2se solenoids at this pressure to preserve lifetime.
float LOXventing    = 30;   // pressure at which ethanol begins venting
#define abortPressure 525   // Cutoff pressure to automatically trigger abort
#define period        0.5   // Sets period for bang-bang control
float sendDelay     = 250;  // Sets frequency of data collection. 1/(sendDelay*10^-3) is frequency in Hz
// END OF USER DEFINED PARAMETERS //
// refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts


//::::::DEFINE INSTRUMENT PINOUTS::::::://

// CLK PIN (SHARED ACROSS ALL HX INSTRUMENTS)
#define CLK 27

// LOX System
#define PT_O1 36 // LOX Tank PT
float PT_O1_Offset = 1.32;
float PT_O1_Slope  = 0.0000412;
// #define PT_O2 39 // LOX Injector PT
// float PT_O2_Offset = 7.25;
// float PT_O2_Slope  = 0.000102;

// Eth System
#define PT_E1 35 // ETH tank PT SWAPPED FROM PINO2
float PT_E1_Offset = -28.97;
float PT_E1_Slope  = .0051;
// #define PT_E2 35 //ETH Injector PT
// float PT_E2_Offset = 2954;
// float PT_E2_Slope  = -1.423;

// Combustion Chamber should be 32, swapped atm
#define PT_C1 32
float PT_C1_Offset = -4.763;
float PT_C1_Slope  = 0.0001055;

// LOADCELLS
#define LC1 33
float LC1_Offset = 10.663;
float LC1_Slope  = 0.0007518;
#define LC2 25
float LC2_Offset = 10.663;
float LC2_Slope  = 0.0007687;
#define LC3 26
float LC3_Offset = 10.663;
float LC3_Slope  = 0.0007951;

// THERMOCOUPLES
#define TC_DO    5
#define TC_DI   17
#define TC_CLK  18
#define TC1_CS  16
#define TC2_CS   4

// CAP SENSORS
// #define CAPSENS1DATA 40
// #define CAPSENS1CLK  52
// #define CAPSENS2DATA 13
// #define CAPSENS2CLK  21

// GPIO expander
#define I2C_SDA 21
#define I2C_SCL 22

// MOSFETS
#define MOSFET_IGNITER   0
#define MOSFET_ETH_MAIN  1
#define MOSFET_EXTRA    10
#define MOSFET_LOX_MAIN  3
#define MOSFET_ETH_PRESS 4
#define MOSFET_LOX_PRESS 5
#define MOSFET_VENT_ETH  6
#define MOSFET_VENT_LOX  2


//::::::DEFINE INSTRUMENTS::::::://

// Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale_PT_O1;
// HX711 scale_PT_O2;
HX711 scale_PT_E1;
// HX711 scale_PT_E2;
HX711 scale_PT_C1;
HX711 scale_LC1;
HX711 scale_LC2;
HX711 scale_LC3;

// Initialize thermocouples.
Adafruit_MAX31855 thermocouple1(TC_CLK, TC1_CS, TC_DO);
Adafruit_MAX31855 thermocouple2(TC_CLK, TC2_CS, TC_DO);

// Initialize mosfets' io expander.
EasyPCF8575 mosfet_pcf;
bool mosfet_pcf_found;

//::::::STATE VARIABLES::::::://
enum STATES {IDLE, ARMED, PRESS, QD, IGNITION, HOTFIRE, ABORT};
string state_names[] = {"Idle", "Armed", "Press", "QD", "Ignition", "HOTFIRE", "Abort"};
int COMState;
int DAQState;
bool ethComplete = false;
bool oxComplete = false;
bool pressComplete = false ;
bool oxVentComplete = false;
bool ethVentComplete = false;

// Delay between loops.
#define IDLE_DELAY 250
#define GEN_DELAY 25


//::::DEFINE READOUT VARIABLES:::://
String serialMessage;
float sendTime;
short int queueLength = 0;

// Define variables to store readings to be sent
int debug_state = 0;
float reading_PT_O1 = 1;
float reading_PT_O2 = 1;
float reading_PT_E1 = 1;
float reading_PT_E2 = 1;
float reading_PT_C1 = 1;
float reading_LC1 = 1;
float reading_LC2 = 1;
float reading_LC3 = 1;
float reading_TC1 = 1;
float reading_TC2 = 1;
float readingCap1 = 0;
float readingCap2 = 0;
short int queueSize = 0;

// Structure example to send data.
// Must match the receiver structure.
typedef struct struct_message {
    int messageTime;
    float pt1;  // PTO1
    float pt2;  // PTO2
    float pt3;  // PTE1
    float pt4;  // PTE2
    float pt5;  // PTC1
    float lc1;
    float lc2;
    float lc3;
    float tc1;
    float tc2;
    int COMState;
    int DAQState;
    short int queueSize;
    bool pressComplete;
    bool ethComplete;
    bool oxComplete;
    // bool oxvent;
    // bool ethVent;
    // bool VentComplete;
} struct_message;

// Create a struct_message called Readings to hold sensor readings
struct_message Readings;
//create a queue for readings in case
struct_message ReadingsQueue[120];

// Create a struct_message to hold incoming commands
struct_message Commands;


//::::::Broadcast Variables::::::://
esp_now_peer_info_t peerInfo;
// REPLACE WITH THE MAC Address of your receiver

// OLD COM BOARD {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C}
// COM BOARD {0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8}
// HEADERLESS BOARD {0x7C, 0x87, 0xCE, 0xF0 0x69, 0xAC}
// NEWEST COM BOARD IN EVA {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
// uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC};
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x50, 0x23, 0x34};
// {0x7C, 0x87, 0xCE, 0xF0, 0x69, 0xAC};
// {0x3C, 0x61, 0x05, 0x4A, 0xD5, 0xE0};
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34};
// Callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  sendTime = millis();
// }

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Commands, incomingData, sizeof(Commands));
  COMState = Commands.COMState;
}


// Initialize all sensors and parameters.
void setup() {
  // pinMode(ONBOARD_LED,OUTPUT);
  Serial.begin(115200);

  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc.

  // HX711.
  scale_PT_O1.begin(PT_O1, CLK); scale_PT_O1.set_gain(64);
  // scale_PT_O2.begin(PT_O2, CLK); scale_PT_O2.set_gain(64);
  scale_PT_E1.begin(PT_E1, CLK); scale_PT_E1.set_gain(64);
  // scale_PT_E2.begin(PT_E2, CLK); scale_PT_E2.set_gain(64);
  scale_PT_C1.begin(PT_C1, CLK); scale_PT_C1.set_gain(64);
  scale_LC1.begin(LC1, CLK); scale_LC1.set_gain(64);
  scale_LC2.begin(LC2, CLK); scale_LC2.set_gain(64);
  scale_LC3.begin(LC3, CLK); scale_LC3.set_gain(64);

  // Thermocouple.
  pinMode(TC1_CS, INPUT);
  pinMode(TC2_CS, INPUT);

  // Serial.println("MAX31855 test");
  // // wait for MAX chip to stabilize
  // delay(500);
  // Serial.print("Initializing sensor...");
  // if (!thermocouple.begin()) {
  //   Serial.println("ERROR.");
  //   while (1) delay(10);
  // }

  // OPTIONAL: Can configure fault checks as desired (default is ALL)
  // Multiple checks can be logically OR'd together.
  // thermocouple.setFaultChecks(MAX31855_FAULT_OPEN | MAX31855_FAULT_SHORT_VCC);  // short to GND fault is ignored

  // Cap Sensor.
  // pinMode(CAPSENS1DATA, INPUT);
  // pinMode(CAPSENS1CLK, OUTPUT);
  // pinMode(CAPSENS2DATA, INPUT);
  // pinMode(CAPSENS2CLK, OUTPUT);

  // MOSFET.
  mosfet_pcf.startI2C(21, 22, SEARCH); // Only SEARCH, if using normal pins in Arduino
  mosfet_pcf_found = false;
  if (!mosfet_pcf.check(SEARCH)) {
    Serial.println("Device not found. Try to specify the address");
    Serial.println(mosfet_pcf.whichAddr());
    while (true); // Why do we have this while true?
  } else {
    mosfet_pcf_found = true;
  }
  mosfetCloseAllValves(); // make sure everything is off by default (Up = Off, Down = On)
  delay(500); // startup time to make sure its good for personal testing

  // Broadcast setup.
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  //Print MAC Accress on startup for easier connections
  Serial.println(WiFi.macAddress());

  // Init ESP-NOWf
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
 if (!WIFIDEBUG) {
  esp_now_register_recv_cb(OnDataRecv);}

  sendTime = millis();
  DAQState = IDLE;
}


//::::::STATE MACHINE::::::://

// Main Structure of State Machine.
void loop() {
  fetchCOMState();
  if (DEBUG) {
    Serial.print(state_names[DAQState]);
  }
  if (DEBUG || COMState == IDLE || COMState == ABORT) {
    syncDAQState();
  }

  logData();

  sendDelay = GEN_DELAY;
  switch (DAQState) {
    case (IDLE):
      sendDelay = IDLE_DELAY;
      if (COMState == ARMED) { syncDAQState(); }
      idle();
      break;

    case (ARMED): // NEED TO ADD TO CASE OPTIONS //ALLOWS OTHER CASES TO TRIGGER //INITIATE TANK PRESS LIVE READINGS
      if (COMState == PRESS) { syncDAQState(); }
      armed();
      break;

    case (PRESS):
      if (COMState == QD/* || COMState == IGNITION*/) { syncDAQState(); } // Does press go directly to ignition?
      press();
      break;

    case (QD):
      if (COMState == IGNITION) { syncDAQState(); }
      quick_disconnect();
      break;

    case (IGNITION):
      if (COMState == HOTFIRE) { syncDAQState(); }
      ignition();
      break;

    case (HOTFIRE):
      hotfire();
      break;

    case (ABORT):
      abort_sequence();
      break;
  }
}

// State Functions.

// Everything should be off.
void idle() {
  // mosfetCloseValve(MOSFET_LOX_MAIN);
  // mosfetCloseValve(MOSFET_ETH_MAIN);
  // mosfetCloseValve(MOSFET_IGNITER);
  // mosfetCloseValve(MOSFET_LOX_PRESS);
  // mosfetCloseValve(MOSFET_ETH_PRESS);
  // mosfetCloseValve(MOSFET_VENT_LOX);
  // mosfetCloseValve(MOSFET_VENT_ETH);
  mosfetCloseAllValves();
}

// Oxygen and fuel should not flow yet.
// This function is the same as idle?
void armed() {
  mosfetCloseValve(MOSFET_LOX_PRESS);
  mosfetCloseValve(MOSFET_ETH_PRESS);
}

void press() {
  if (reading_PT_O1 < pressureOx*threshold || reading_PT_E1 < pressureFuel*threshold) {
    oxComplete = false;
    ethComplete = false;

    while (!oxComplete || !ethComplete) {
      // Serial.print("IN press WHILE LOOP");
      logData();

      if (reading_PT_O1 < pressureOx) { // Should it be pressureOx*threshold?
        mosfetOpenValve(MOSFET_LOX_PRESS);
        if (DEBUG) {
          reading_PT_O1 += 0.1;
        }
      } else {
        mosfetCloseValve(MOSFET_LOX_PRESS);
        oxComplete = true;
      }
      if (reading_PT_E1 < pressureFuel) {
        mosfetOpenValve(MOSFET_ETH_PRESS);
        if (DEBUG) {
          reading_PT_E1 += 0.2;
        }
      } else {
        mosfetCloseValve(MOSFET_ETH_PRESS);
        ethComplete = true;
      }

      // ABORT CASES
      CheckAbort();
    }
  }
  CheckAbort();
}

void ignition() {
  mosfetOpenValve(MOSFET_IGNITER);
}

void hotfire() {
  mosfetOpenValve(MOSFET_LOX_MAIN);
  mosfetOpenValve(MOSFET_ETH_MAIN);
}

// Disconnect harnessings and check state of rocket.
void quick_disconnect() {
  mosfetCloseValve(MOSFET_LOX_PRESS);
  mosfetCloseValve(MOSFET_ETH_PRESS);
  // TODO: QD code here
  CheckAbort();
}

void abort_sequence() {
  // mosfetOpenValve(MOSFET_VENT_LOX);
  // mosfetOpenValve(MOSFET_VENT_ETH);
  // Waits for LOX pressure to decrease before venting Eth through pyro
  mosfetCloseValve(MOSFET_LOX_PRESS);
  mosfetCloseValve(MOSFET_ETH_PRESS);

  int currtime = millis();
  oxVentComplete = !(reading_PT_O1 > 1.3*ventTo); // 1.3 is magic number.
  ethVentComplete = !(reading_PT_E1 > 1.3*ventTo);
  while(!(oxVentComplete && ethVentComplete)){
    getReadings();
    printSensorReadings();
    if (reading_PT_O1 > LOXventing) { // vent only lox down to loxventing pressure
      mosfetOpenValve(MOSFET_VENT_LOX);
      if (DEBUG) {
        reading_PT_O1 = reading_PT_O1 - 0.25;
      }
    } else {
      if (reading_PT_E1 > ventTo) {
        mosfetOpenValve(MOSFET_VENT_ETH); // vent ethanol
        if (DEBUG) {
          reading_PT_E1 = reading_PT_E1 - 0.2;
        }
      } else {
        mosfetCloseValve(MOSFET_VENT_ETH);
        ethVentComplete = true;
      }

      if (reading_PT_O1 > ventTo) {
        mosfetOpenValve(MOSFET_VENT_LOX); // vent lox
        if (DEBUG) {
          reading_PT_O1 = reading_PT_O1 - 0.1;
        }
      } else { // lox vented to acceptable hold pressure
        mosfetCloseValve(MOSFET_VENT_LOX); // close lox
        oxVentComplete = true;
      }
    }
  }
}

// Helper Functions

// Get commanded state from COM board.
void fetchCOMState() {
  if (Serial.available() > 0) {
    COMState = Serial.read() - 48;
    delay(50);
    Serial.println(COMState);
  }
}

// Sync state of DAQ board with COM board.
void syncDAQState() {
  DAQState = COMState;
}

void CheckAbort() {
  if (COMState == ABORT || reading_PT_O1 >= abortPressure || reading_PT_E1 >= abortPressure) {
    mosfetCloseValve(MOSFET_ETH_PRESS);
    mosfetCloseValve(MOSFET_LOX_PRESS);
    DAQState = ABORT;
  }
}

void mosfetCloseAllValves(){
  if (mosfet_pcf_found) {
    mosfet_pcf.setAllBitsUp();
  }
}
void mosfetCloseValve(int num){
  if (mosfet_pcf_found) {
    mosfet_pcf.setLeftBitUp(num);
  }
}
void openSolenoidFuel(int num){
  if (mosfet_pcf_found) {
    mosfet_pcf.setLeftBitDown(num);
  }
}


//::::::DATA LOGGING AND COMMUNICATION::::::://
void logData() {
  getReadings();
  printSensorReadings();
  if (millis()-sendTime > sendDelay) {
    sendTime = millis();
    sendData();
    // saveData();
  }
}

void getReadings(){
  if (!DEBUG) {
    reading_PT_O1 = PT_O1_Slope * scale_PT_O1.read() + PT_O1_Offset;
    // reading_PT_O2 = PT_O2_Slope * scale_PT_O2.read() + PT_O2_Offset;
    reading_PT_E1 = PT_E1_Slope * scale_PT_E1.read() + PT_E1_Offset;
    // reading_PT_E2 = PT_E2_Slope * scale_PT_E2.read() + PT_E2_Offset;
    reading_PT_C1 = PT_C1_Slope * scale_PT_C1.read() + PT_C1_Offset;
    reading_LC1   = LC1_Slope   * scale_LC1.read()   + LC1_Offset;
    reading_LC2   = LC2_Slope   * scale_LC2.read()   + LC2_Offset;
    reading_LC3   = LC3_Slope   * scale_LC3.read()   + LC3_Offset;
    // reading_TC1 = thermocouple1.readCelsius();
    // reading_TC2 = thermocouple2.readCelsius();
    // readingCap1 = analogRead(CAPSENS1DATA);
    // readingCap2 = analogRead(CAPSENS2DATA);
  }
}

void printSensorReadings() {
  serialMessage  = millis()         + " ";
  serialMessage += reading_PT_O1    + " ";
  serialMessage += reading_PT_O2    + " ";
  serialMessage += reading_PT_E1    + " ";
  serialMessage += reading_PT_E2    + " ";
  serialMessage += reading_PT_C1    + " ";
  serialMessage += reading_LC1      + " ";
  serialMessage += reading_LC2      + " ";
  serialMessage += reading_LC3      + " ";
  serialMessage += reading_TC1      + " ";
  serialMessage += reading_TC2      + " ";
  serialMessage += DAQState         + " ";
  serialMessage += "Queue Length: " + queueLength;
  Serial.println(serialMessage);
}

void sendData() {
  addReadingsToQueue();
  sendQueue();
}

void addReadingsToQueue() {
  if (queueLength < 40) {
    queueLength += 1;
    ReadingsQueue[queueLength].messageTime = millis();
    ReadingsQueue[queueLength].pt1         = reading_PT_O1;
    ReadingsQueue[queueLength].pt2         = reading_PT_O2;
    ReadingsQueue[queueLength].pt3         = reading_PT_E1;
    ReadingsQueue[queueLength].pt4         = reading_PT_E2;
    ReadingsQueue[queueLength].pt5         = reading_PT_C1;
    ReadingsQueue[queueLength].lc1         = reading_LC1;
    ReadingsQueue[queueLength].lc2         = reading_LC2;
    ReadingsQueue[queueLength].lc3         = reading_LC3;
    ReadingsQueue[queueLength].tc1         = reading_TC1;
    ReadingsQueue[queueLength].tc2         = reading_TC2;
    ReadingsQueue[queueLength].queueSize   = queueLength;
    ReadingsQueue[queueLength].DAQState    = DAQState;
    ReadingsQueue[queueLength].oxComplete  = oxComplete;
    ReadingsQueue[queueLength].ethComplete = ethComplete;
  }
}

void sendQueue() {
  if (queueLength < 0) {
    return;
  }
  // Set values to send
  Readings = ReadingsQueue[queueLength];

  if (!WIFIDEBUG) {
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));

    if (result == ESP_OK) {
      Serial.println("Sent with success Data Send");
      // ReadingsQueue[queueLength].pt1val = 0;
      queueLength -= 1;
    } else {
      Serial.println("Error sending the data");
    }
  }
}
