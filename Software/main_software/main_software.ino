#include <SPI.h>
#include <MFRC522.h>
#include <EEPROM.h>
#include <vector>
#include <array>
#include <algorithm> // For std::find

// --- Pin Definitions ---
#define SS_PIN D7
#define RST_PIN D6
#define BTNUP D1
#define BTNDOWN D0
#define MOTOR_A D4
#define MOTOR_B D5
#define ENC_A D2 // This pin MUST support interrupts
#define ENC_B D3

// --- Constants ---
const int MOTOR_SPEED_FULL = 255;
const int MOTOR_SPEED_MIN = 25;//minimum speed for it to move.
const int MOTOR_SPEED_MANUAL = 100;
const int FULL_REVOLUTIION_TICKS = 1200;//12 ticks per encoder revolution, 100:1 reduction
const int DOOR_TRAVEL_TICKS = 400;
const long DOOR_HOLD_TIME = 2000;  // ms
const int MAX_IDS = 50;           // Set a reasonable max limit for IDs

const int Kp = 3;
const int TARGET_RANGE = 20;// Number of tick+- to consider in target.

// --- EEPROM Data Structure ---
// EEPROM is emulated Flash memory on the RP2040.
// Layout: [MagicByte1, MagicByte2, ID_Count, ID1, ID2, ...]
const int EEPROM_SIZE = 2 + 1 + (MAX_IDS * 4); // 2 magic bytes, 1 count byte, 4 bytes per ID
const byte EEPROM_MAGIC_1 = 0xDE;
const byte EEPROM_MAGIC_2 = 0xAD;
#define EEPROM_ADDR_COUNT 2
#define EEPROM_ADDR_IDS_START 3

// --- RFID Instance ---
MFRC522 rfid(SS_PIN, RST_PIN);

// --- Global Variables ---
std::vector<std::array<byte, 4>> acceptedIDsList; // Holds IDs in memory

byte nuidPICC[4];    // Stores the NUID of the last scanned card
bool isOpen = false; // Tracks door state

volatile long encoderPos = 0; // Current encoder position
long targetPos = 0;           // Target encoder position
unsigned long openTime = 0;   // Timestamp for door opening

bool programmingMode = false; // Flag for ID management mode
unsigned long buttonHoldStartTime = 0;

enum motorDirection { forward, backward };

// --- Function Prototypes ---
void loadIDsFromEEPROM();
void saveIDsToEEPROM();

// --- Initialization ---
void setup() {
  Serial.begin(9600);

  // Init EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadIDsFromEEPROM();

  // Init RFID
  SPI.begin();
  rfid.PCD_Init();

  // Pin Modes
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  pinMode(BTNUP, INPUT_PULLUP);
  pinMode(BTNDOWN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Attach encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);;

  Serial.println(F("\n--- RFID Door Lock Ready ---"));
  Serial.printf("Loaded IDs from EEPROM.\n");
}

// --- Main Loop ---
void loop() {
  bool btnUpState = (digitalRead(BTNUP) == LOW);
  bool btnDownState = (digitalRead(BTNDOWN) == LOW);

  // --- Mode Switching Logic ---
  if (btnUpState && btnDownState) {
    if (buttonHoldStartTime == 0) {
      buttonHoldStartTime = millis();
    }
    // Enter programming mode after a 2-second hold
    if (!programmingMode && (millis() - buttonHoldStartTime > 2000)) {
      programmingMode = true;
      driveMotor(0); // Ensure motor is off
      targetPos = encoderPos; // Cancel any movement
      Serial.println(F("\n--- ENTERING PROGRAMMING MODE ---"));
      Serial.println(F("Scan a tag to add/remove. Press any single button to exit."));
    }
  } else {
    buttonHoldStartTime = 0; // Reset timer if buttons aren't held
    // Exit programming mode if a single button is pressed
    if (programmingMode && (btnUpState || btnDownState)) {
      programmingMode = false;
      Serial.println(F("--- EXITING PROGRAMMING MODE ---\n"));
      delay(500); // Debounce exit
    }
  }
  
  // --- Main Logic based on Mode ---
  if (programmingMode) {
    // In programming mode, we only check for cards to manage them
    if (readCard()) {
      manageScannedID();
    }
  } else {
    // --- Normal Operation Mode ---
    
    // A static variable retains its value between loop cycles.
    // This helps us track if we were just manually driving the motor.
    static bool wasManualDriving = false;

    if (btnUpState) {
      targetPos = encoderPos; // Cancel any automatic movement
      driveMotor(MOTOR_SPEED_MANUAL);
      wasManualDriving = true;
    } else if (btnDownState) {
      targetPos = encoderPos; // Cancel any automatic movement
      driveMotor(-MOTOR_SPEED_MANUAL);
      wasManualDriving = true;
    } else {
      // This block runs when no manual buttons are being pressed.

      // If we just finished a manual drive (i.e., the button was just released)...
      if (wasManualDriving) {
        driveMotor(0);
        // Set the new zero position ON RELEASE, which is more intuitive.
        encoderPos = 0;
        targetPos = 0;
        wasManualDriving = false; // Reset the flag
      }

      // Run automatic logic only when not in a manual-drive state.
      // Only check for new cards if the motor is idle.
      if (targetReached()) {
        if (readCard() && verifyID(nuidPICC)) {
          if (!isOpen) {
            targetPos = DOOR_TRAVEL_TICKS;
            isOpen = true;
            openTime = 0;
            Serial.println(F("Opening door"));
          }
        }
      }

      // motorLoop handles automatic movement towards a target (e.g., after a card scan)
      motorLoop();

      if (isOpen) {
        if (openTime == 0) {
          openTime = millis();
        }
        if (millis() - openTime > DOOR_HOLD_TIME) {
          targetPos = 0;

          isOpen = false;
          openTime = 0;
          Serial.println(F("Closing door"));
          
        }
      }
    }
  }

  //debug encoders
  byte currentState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  Serial.printf("encoder state %d \n",currentState);

}

bool targetReached() {
  return (encoderPos <= targetPos + TARGET_RANGE) && (encoderPos >= targetPos - TARGET_RANGE);
}

// --- ID & EEPROM Management ---
void manageScannedID() {
  std::array<byte, 4> scannedID;
  memcpy(scannedID.data(), nuidPICC, 4);

  // Check if the ID already exists in our list
  auto it = std::find(acceptedIDsList.begin(), acceptedIDsList.end(), scannedID);

  if (it != acceptedIDsList.end()) {
    // ID exists, so remove it
    acceptedIDsList.erase(it);
    Serial.print(F("ID Removed: "));
    printHex(nuidPICC, 4);
    Serial.println();
    saveIDsToEEPROM();
  } else {
    // ID does not exist, so add it (if we have space)
    if (acceptedIDsList.size() < MAX_IDS) {
      acceptedIDsList.push_back(scannedID);
      Serial.print(F("ID Added: "));
      printHex(nuidPICC, 4);
      Serial.println();
      saveIDsToEEPROM();
    } else {
      Serial.println(F("Cannot add ID, list is full."));
    }
  }
}

void loadIDsFromEEPROM() {
  acceptedIDsList.clear();
  if (EEPROM.read(0) == EEPROM_MAGIC_1 && EEPROM.read(1) == EEPROM_MAGIC_2) {
    byte idCount = EEPROM.read(EEPROM_ADDR_COUNT);
    for (byte i = 0; i < idCount; i++) {
      std::array<byte, 4> id;
      int startAddr = EEPROM_ADDR_IDS_START + (i * 4);
      for (byte j = 0; j < 4; j++) {
        id[j] = EEPROM.read(startAddr + j);
      }
      acceptedIDsList.push_back(id);
    }
  } else {
    // EEPROM not formatted, format it now
    Serial.println(F("EEPROM not initialized. Formatting..."));
    saveIDsToEEPROM(); // This will write magic bytes and count of 0
  }
}

void saveIDsToEEPROM() {
  EEPROM.write(0, EEPROM_MAGIC_1);
  EEPROM.write(1, EEPROM_MAGIC_2);
  EEPROM.write(EEPROM_ADDR_COUNT, acceptedIDsList.size());
  for (size_t i = 0; i < acceptedIDsList.size(); i++) {
    int startAddr = EEPROM_ADDR_IDS_START + (i * 4);
    for (byte j = 0; j < 4; j++) {
      EEPROM.write(startAddr + j, acceptedIDsList[i][j]);
    }
  }
  if (EEPROM.commit()) {
    Serial.println(F("EEPROM saved successfully."));
  } else {
    Serial.println(F("ERROR: EEPROM commit failed!"));
  }
}

bool verifyID(byte id[4]) {
  std::array<byte, 4> targetID;
  memcpy(targetID.data(), id, 4);
  
  for (const auto& acceptedID : acceptedIDsList) {
    if (acceptedID == targetID) {
      Serial.println(F("ID Accepted."));
      return true;
    }
  }
  
  Serial.println(F("ID Rejected."));
  return false;
}

// --- Motor, RFID, and Helper functions ---


void driveMotor(int speed) {
  if (speed > 0) {
    analogWrite(MOTOR_A, speed);
    digitalWrite(MOTOR_B, LOW);
  } else if (speed < 0) {
    digitalWrite(MOTOR_A, LOW);
    analogWrite(MOTOR_B, -speed);
  } else {
    digitalWrite(MOTOR_A, LOW);
    digitalWrite(MOTOR_B, LOW);
  }
}

void motorLoop() {
  // Currently I need to use a PID controller, but this is a proportional controller
  int error = targetPos - encoderPos;
  // Equation, if a full revolution is needed it will go full speed, Kp disregarded
  int motorSpeed = Kp * error * (MOTOR_SPEED_FULL / FULL_REVOLUTIION_TICKS);

  //bounding the motor speed between full speed and minimum move speed.
  motorSpeed = max(min(motorSpeed, MOTOR_SPEED_FULL), MOTOR_SPEED_MIN);

  if (targetReached()) {
    motorSpeed = 0;
  }

  driveMotor(motorSpeed);
  
}

bool readCard() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return false;
  }
  memcpy(nuidPICC, rfid.uid.uidByte, 4);
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  return true;
}

// QUADRATURE ENCODER FUNCTION
void readEncoder() {
  // A static variable to remember the last state of the encoder pins
  static byte lastState = 0;
  
  // Read the current state of the two pins
  byte currentState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  Serial.printf("encoder state",currentState);

  // If the state hasn't changed, do nothing (prevents floating pins from causing issues)
  if (currentState == lastState) {
    return;
  }
  
  // A lookup table is the most efficient way to decode quadrature states.
  // The index is calculated from the previous and current states.
  // Values: 0 = No change, 1 = Clockwise, -1 = Counter-Clockwise
  static const int8_t lookupTable[] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
  };
  
  int8_t direction = lookupTable[(lastState << 2) | currentState];
  encoderPos += direction;
  
  // Save the current state for the next time the interrupt fires
  lastState = currentState;
}

void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}