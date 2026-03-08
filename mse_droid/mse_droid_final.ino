#include <Bluepad32.h>
#include "DFRobotDFPlayerMini.h"
#include "HardwareSerial.h"

// Pin Connections
int builtInLed = 2;
int df_player_busy_pin = 36;
int shift_ser_input = 21;
int shift_shift_srclk = 19;
int shift_latch_rclk = 18;

// MOTOR 1 - Front 
int ENApin_motor_1 = 14; // Motor 1 speed
int IN1pin_motor_1 = 27; // Motor 1 dir1
int IN2pin_motor_1 = 26; // Motor 1 dir2

// MOTOR 2 - Front 
int ENBpin_motor_2 = 32; // Motor 2 speed
int IN3pin_motor_2 = 25; // Motor 2 dir1
int IN4pin_motor_2 = 33; // Motor 2 dir2

// MOTOR 3 - Back 
int ENApin_motor_3 = 5;  // Motor 3 speed
int IN1pin_motor_3 = 12; // Motor 3 dir1
int IN2pin_motor_3 = 13; // Motor 3 dir2

// MOTOR 4 - Back 
int ENBpin_motor_4 = 4;  // Motor 4 speed
int IN3pin_motor_4 = 23; // Motor 4 dir1
int IN4pin_motor_4 = 22; // Motor 4 dir2

// Initalize variables
int motorSpeed = 0;
int reverseSpeed = 0;

// DFPlayerMini Variables
HardwareSerial mySerial(2);
DFRobotDFPlayerMini player;

// Sound Variables
unsigned long lastEventTime = 0;
unsigned long nextInterval = 0;
const unsigned long MIN_INTERVAL = 5000;   // 5 seconds
const unsigned long MAX_INTERVAL = 30000;  // 30 seconds
int soundPlaying = 0;

// LED Variables
unsigned long lastFlickerTime = 0;
unsigned long flickerInterval = 100;  // initial delay

// Controller Variables
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
uint16_t lastButtons = 0;
uint8_t lastDpad = 0;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
      }
    }

    if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
  "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ()        // Accelerometer Z
  );
}

void playSound(){
  int sound = random(1, 24);
  player.play(sound);
  Serial.print("Now playing track: ");
  Serial.println(sound);
}

void setSpeed(int leftSpeed, int rightSpeed){
  analogWrite(ENApin_motor_1, leftSpeed);
  analogWrite(ENApin_motor_3, leftSpeed);

  analogWrite(ENBpin_motor_2, rightSpeed);
  analogWrite(ENBpin_motor_4, rightSpeed);
}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
  uint16_t currentButtons = ctl->buttons();
  uint8_t currentDpad = ctl->dpad();
 
  // ===== BUTTONS ===== //

  //== PS4 X button ==//
  if (currentButtons & 0x0001) {
    digitalWrite(builtInLed, HIGH);
  } else {
    digitalWrite(builtInLed, LOW);
  }

  //== PS4 Square button (0x0004) ==//
  if ((currentButtons & 0x0004) && !(lastButtons & 0x0004)) {
    // square JUST pressed
  }

  if (!(currentButtons & 0x0004) && (lastButtons & 0x0004)) {
    // square JUST released
  }

  //== PS4 Triangle button ==//
  if ((currentButtons & 0x0008) && !(lastButtons & 0x0008)) {
    playSound();
  }

  if (!(currentButtons & 0x0008) && (lastButtons & 0x0008)) {
    // triangle JUST released
  }

  //== PS4 Circle button (0x0002) ==//
  if ((currentButtons & 0x0002) && !(lastButtons & 0x0002)) {
    // circle JUST pressed
  }

  if (!(currentButtons & 0x0002) && (lastButtons & 0x0002)) {
    // circle JUST released
  }

  //== PS4 R1 button (0x0020) ==//
  if ((currentButtons & 0x0020) && !(lastButtons & 0x0020)) {
    // R1 JUST pressed
  }

  if (!(currentButtons & 0x0020) && (lastButtons & 0x0020)) {
    // R1 JUST released
  }

  //== PS4 L1 button (0x0010) ==//
  if ((currentButtons & 0x0010) && !(lastButtons & 0x0010)) {
    // L1 JUST pressed
  }

  if (!(currentButtons & 0x0010) && (lastButtons & 0x0010)) {
    // L1 JUST released
  }

  // ===== DPAD ===== //
  //== Dpad UP ==//
  if ((currentDpad == 0x01) && (lastDpad != 0x01)) {
    int currentVol = player.readVolume();
    currentVol = constrain(currentVol + 5, 0, 30);
    player.volume(currentVol);

    Serial.print("Current Volume: ");
    Serial.println(currentVol);
  }

  if ((currentDpad != 0x01) && (lastDpad == 0x01)) {
    // UP JUST released
  }

  //== Dpad DOWN ==//
  if ((currentDpad == 0x02) && (lastDpad != 0x02)) {
    int currentVol = player.readVolume();
    currentVol = constrain(currentVol - 5, 0, 30);
    player.volume(currentVol);

    Serial.print("Current Volume: ");
    Serial.println(currentVol);
  }

  if ((currentDpad != 0x02) && (lastDpad == 0x02)) {
    // DOWN JUST released
  }

  //== Dpad LEFT (0x08) ==//
  if ((currentDpad == 0x08) && (lastDpad != 0x08)) {
    // LEFT JUST pressed
  }

  if ((currentDpad != 0x08) && (lastDpad == 0x08)) {
    // LEFT JUST released
  }

  //== Dpad RIGHT (0x04) ==//
  if ((currentDpad == 0x04) && (lastDpad != 0x04)) {
    // RIGHT JUST pressed
  }

  if ((currentDpad != 0x04) && (lastDpad == 0x04)) {
    // RIGHT JUST released
  }

  // ====== MOTOR LOGIC =====

  int r2 = ctl->throttle(); 
  motorSpeed = map(r2, 0, 1023, 0, 255);

  int l2 = ctl->brake(); 
  reverseSpeed = map(l2, 0, 1023, 0, 255);

  if (r2 > 0){

    //== LEFT JOYSTICK - DOWN ==//
    if (ctl->axisY() >= 25) {
      digitalWrite(IN1pin_motor_1, LOW);
      digitalWrite(IN2pin_motor_1, HIGH);
      digitalWrite(IN3pin_motor_2, LOW);
      digitalWrite(IN4pin_motor_2, HIGH);
      digitalWrite(IN1pin_motor_3, LOW);
      digitalWrite(IN2pin_motor_3, HIGH);
      digitalWrite(IN3pin_motor_4, LOW);
      digitalWrite(IN4pin_motor_4, HIGH);

      setSpeed(motorSpeed, motorSpeed);
    }

    //== RIGHT JOYSTICK - LEFT ==//
    else if (ctl->axisRX() <= -100) {
      digitalWrite(IN3pin_motor_2, HIGH);
      digitalWrite(IN4pin_motor_2, LOW);
      digitalWrite(IN3pin_motor_4, HIGH);
      digitalWrite(IN4pin_motor_4, LOW);

      setSpeed(0, motorSpeed);
    }

    //== RIGHT JOYSTICK - RIGHT ==//
    else if (ctl->axisRX() >= 100) {
      digitalWrite(IN1pin_motor_1, HIGH);
      digitalWrite(IN2pin_motor_1, LOW);
      digitalWrite(IN1pin_motor_3, HIGH);
      digitalWrite(IN2pin_motor_3, LOW);

      setSpeed(motorSpeed, 0);
    }

    else {
      digitalWrite(IN1pin_motor_1, HIGH);
      digitalWrite(IN2pin_motor_1, LOW);
      digitalWrite(IN3pin_motor_2, HIGH);
      digitalWrite(IN4pin_motor_2, LOW);
      digitalWrite(IN1pin_motor_3, HIGH);
      digitalWrite(IN2pin_motor_3, LOW);
      digitalWrite(IN3pin_motor_4, HIGH);
      digitalWrite(IN4pin_motor_4, LOW);

      setSpeed(motorSpeed, motorSpeed);
    }
  }

  else if (l2 > 0){
    //== RIGHT JOYSTICK - LEFT ==//
    if (ctl->axisRX() <= -100) {
      digitalWrite(IN1pin_motor_1, LOW);
      digitalWrite(IN2pin_motor_1, HIGH);
      digitalWrite(IN1pin_motor_3, LOW);
      digitalWrite(IN2pin_motor_3, HIGH);

      setSpeed(reverseSpeed, 0);
    }

    //== RIGHT JOYSTICK - RIGHT ==//
    else if (ctl->axisRX() >= 100) {
      digitalWrite(IN3pin_motor_2, LOW);
      digitalWrite(IN4pin_motor_2, HIGH);
      digitalWrite(IN3pin_motor_4, LOW);
      digitalWrite(IN4pin_motor_4, HIGH);

      setSpeed(0, reverseSpeed);
    }

    else{
      digitalWrite(IN1pin_motor_1, LOW);
      digitalWrite(IN2pin_motor_1, HIGH);
      digitalWrite(IN3pin_motor_2, LOW);
      digitalWrite(IN4pin_motor_2, HIGH);
      digitalWrite(IN1pin_motor_3, LOW);
      digitalWrite(IN2pin_motor_3, HIGH);
      digitalWrite(IN3pin_motor_4, LOW);
      digitalWrite(IN4pin_motor_4, HIGH);

      setSpeed(reverseSpeed, reverseSpeed);
    }
  }

  else{
    setSpeed(0, 0);
  }

  lastButtons = currentButtons;
  lastDpad = currentDpad;
  // dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(builtInLed, OUTPUT);
  pinMode(df_player_busy_pin, INPUT);
  pinMode(shift_ser_input, OUTPUT);
  pinMode(shift_shift_srclk, OUTPUT);
  pinMode(shift_latch_rclk, OUTPUT);

  pinMode(IN1pin_motor_1, OUTPUT);
  pinMode(IN2pin_motor_1, OUTPUT);
  pinMode(ENApin_motor_1, OUTPUT);

  pinMode(IN3pin_motor_2, OUTPUT);
  pinMode(IN4pin_motor_2, OUTPUT);
  pinMode(ENBpin_motor_2, OUTPUT);

  pinMode(IN1pin_motor_3, OUTPUT);
  pinMode(IN2pin_motor_3, OUTPUT);
  pinMode(ENApin_motor_3, OUTPUT);

  pinMode(IN3pin_motor_4, OUTPUT);
  pinMode(IN4pin_motor_4, OUTPUT);
  pinMode(ENBpin_motor_4, OUTPUT);

  digitalWrite(builtInLed, HIGH);

  // Direction pins LOW
  digitalWrite(IN1pin_motor_1, LOW);
  digitalWrite(IN2pin_motor_1, LOW);
  digitalWrite(IN3pin_motor_2, LOW);
  digitalWrite(IN4pin_motor_2, LOW);
  digitalWrite(IN1pin_motor_3, LOW);
  digitalWrite(IN2pin_motor_3, LOW);
  digitalWrite(IN3pin_motor_4, LOW);
  digitalWrite(IN4pin_motor_4, LOW);

  // Disable with PWM
  analogWrite(ENApin_motor_1, 0);
  analogWrite(ENBpin_motor_2, 0);
  analogWrite(ENApin_motor_3, 0);
  analogWrite(ENBpin_motor_4, 0);

  delay(5000);

  mySerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.begin(115200);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  if (!player.begin(mySerial)){
    Serial.println("DFPlayer not detected");
    while(true);
  }

  Serial.println("DFPlayer detected");
  digitalWrite(builtInLed, LOW);

  player.setTimeOut(500);  // Serial timeout 500ms
  player.volume(10);       // Set volume to max 
  player.EQ(0);            // Normal equalization

  randomSeed(analogRead(0));  // seed randomness
  nextInterval = random(MIN_INTERVAL, MAX_INTERVAL);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();
  
  if (millis() - lastEventTime >= nextInterval) {
    playSound();
    lastEventTime = millis();
    nextInterval = random(MIN_INTERVAL, MAX_INTERVAL); 
  }

  soundPlaying = digitalRead(df_player_busy_pin);

  // ===== Flicker LEDs While Track is Playing =====
  if (soundPlaying == LOW) {
      if (millis() - lastFlickerTime >= flickerInterval) {

          // generate random flicker value
          byte flicker = random(0, 256);

          // shift register update
          digitalWrite(shift_latch_rclk, LOW);
          shiftOut(shift_ser_input, shift_shift_srclk, MSBFIRST, flicker);
          digitalWrite(shift_latch_rclk, HIGH);

          // ===== timing updates =====
          lastFlickerTime = millis();
          flickerInterval = random(100, 500);
      }
  } 
  else { // Track is done playing → clear LEDs
      digitalWrite(shift_latch_rclk, LOW);
      shiftOut(shift_ser_input, shift_shift_srclk, MSBFIRST, 0x00); // all LEDs off
      digitalWrite(shift_latch_rclk, HIGH);

      lastFlickerTime = millis();
      flickerInterval = random(100, 500);
  }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  vTaskDelay(1);
  // delay(150);
}
