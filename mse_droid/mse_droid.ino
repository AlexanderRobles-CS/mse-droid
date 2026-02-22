#include <Bluepad32.h>
#include "DFRobotDFPlayerMini.h"
#include "HardwareSerial.h"

// Pin Connections
int builtInLed = 2;

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

HardwareSerial mySerial(2);
DFRobotDFPlayerMini player;
String line;
char command;
int isPaused = 0;
int repeat = 0;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

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

void setSpeed(int leftSpeed, int rightSpeed){
  analogWrite(ENApin_motor_1, leftSpeed);
  analogWrite(ENApin_motor_3, leftSpeed);

  analogWrite(ENBpin_motor_2, rightSpeed);
  analogWrite(ENBpin_motor_4, rightSpeed);
}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...
 
  //== PS4 X button = 0x0001 ==//
  if (ctl->buttons() == 0x0001) {
    // code for when X button is pushed
    digitalWrite(builtInLed, HIGH);
  }
  if (ctl->buttons() != 0x0001) {
    // code for when X button is released
    digitalWrite(builtInLed, LOW);
  }

  //== PS4 Square button = 0x0004 ==//
  if (ctl->buttons() == 0x0004) {
    // code for when square button is pushed
  }
  if (ctl->buttons() != 0x0004) {
  // code for when square button is released
  }

  //== PS4 Triangle button = 0x0008 ==//
  if (ctl->buttons() == 0x0008) {
    int sound = random(1, 24);
    player.play(sound);
    Serial.print("Now playing track: ");
    Serial.println(sound); 
  }

  if (ctl->buttons() != 0x0008) {
    // code for when triangle is released
  }

  //== PS4 Circle button = 0x0002 ==//
  if (ctl->buttons() == 0x0002) {
    // code for when circle button is pushed
  }
  if (ctl->buttons() != 0x0002) {
    // code for when circle button is released
  }

  //== PS4 Dpad UP button = 0x01 ==//
  if (ctl->dpad() == 0x01) {
    int currentVol = player.readVolume();
    if (currentVol < 30) {
      player.volume(currentVol + 1);
    }
    Serial.print("Current Volume: ");
    Serial.println(player.readVolume());
  }

  if (ctl->dpad() != 0x01) {
    // code for when dpad up is released
  }

  //==PS4 Dpad DOWN button = 0x02==//
  if (ctl->dpad() == 0x02) {
    int currentVol = player.readVolume();
    if (currentVol > 0) {
      player.volume(currentVol - 1);
    }
    Serial.print("Current Volume: ");
    Serial.println(player.readVolume());
  }
  if (ctl->dpad() != 0x02) {
    // code for when dpad down button is released
  }

  //== PS4 Dpad LEFT button = 0x08 ==//
  if (ctl->dpad() == 0x08) {
    // code for when dpad left button is pushed
  }
  if (ctl->dpad() != 0x08) {
    // code for when dpad left button is released
  }

  //== PS4 Dpad RIGHT button = 0x04 ==//
  if (ctl->dpad() == 0x04) {
    // code for when dpad right button is pushed
  }
  if (ctl->dpad() != 0x04) {
    // code for when dpad right button is released
  }

  //== PS4 R1 trigger button = 0x0020 ==//
  if (ctl->buttons() == 0x0020) {
    // code for when R1 button is pushed
  }
  if (ctl->buttons() != 0x0020) {
    // code for when R1 button is released
  }

  //== PS4 L1 trigger button = 0x0010 ==//
  if (ctl->buttons() == 0x0010) {
    // code for when L1 button is pushed
  }
  if (ctl->buttons() != 0x0010) {
    // code for when L1 button is released
  }

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

  dumpGamepad(ctl);
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
  // Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  if (!player.begin(mySerial)){
    Serial.println("DFPlayer not detected");
    while(true);
  }

  Serial.println("DFPlayer detected");
  digitalWrite(builtInLed, LOW);

  player.setTimeOut(500);  // Serial timeout 500ms
  player.volume(15);       // Volume 5
  player.EQ(0);            // Normal equalization

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
  
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
  delay(150);
}
