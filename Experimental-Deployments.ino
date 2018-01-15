


#include <SoftwareSerial.h>
#include <MiniMaestroService.h>
#include <TalonSR.h>
#include <PololuG2.h>
#include <HolonomicDrive.h>
#include <PS2X_lib.h>
#include <HS485.h>
#include <NetworkTable.h>
#include <AStar32U4.h>
// Blue #1 PSS_RX FIX - Mode of last 3 values selected.
float pss_rx_values[3];
unsigned char pss_rx_iterator;
// MiniMaestroServices construction
SoftwareSerial maestroSerial(22, 23); // Connect A1 to Maestro's RX. A0 must remain disconnected.
MiniMaestroService maestro(maestroSerial);

// Subsystems construction
HS485 chamber = HS485(maestro, 0); // Some objects can use MiniMaestroService to controll devices. See docs.
HS485 intake = HS485(maestro, 1);
HS485 shooter_servo = HS485(maestro, 3);
float shooter_pos = 120;
TalonSR shooter = TalonSR(maestro, 2);
PololuG2 intakemotor = PololuG2(2, 3, 4);

// Drive base construction
PololuG2 motor1 = PololuG2(maestro, 6, 7, 8, true);
PololuG2 motor2 = PololuG2(maestro, 9, 10, 11, true);
PololuG2 motor3 = PololuG2(maestro, 12, 13, 14, true);
PololuG2 motor4 = PololuG2(maestro, 15, 16, 17, true);

// Can construct drive bases using any speed controllers extended from Motor class. (TalonSR and PololuG2).
HolonomicDrive chassis = HolonomicDrive(motor1, motor2, motor3, motor4);

// Network & Controller construction
PS2X ps2x;
NetworkTable network = NetworkTable(10, 10);
PacketSerial myPacketSerial;

float leftvalue = 0;
float rightvalue = 0;
unsigned long last_blink;
unsigned long last_update;
void setup() {
  // prevents devices from actuating on startup.
  delay(1000);
  // initialize MiniMaestroService communication
  maestroSerial.begin(115200);

  // enable ps2x networking
  ps2x.config_gamepad();
  ps2x.read_gamepad();

  // enable debug usb (Serial) and radio serial (Serial1)
  Serial.begin(115200);
  Serial1.begin(115200);

  // binds radio PacketSerial(encoding&decoding services) to NetworkTable class. Uses lambda expressions.
  myPacketSerial.setStream(&Serial1);
  myPacketSerial.setPacketHandler([](const uint8_t* buffer, size_t size) {
    network.processPacketFromSender(myPacketSerial, buffer, size);
  });

  // sets ps2x controlls.
  network.setPS2(ps2x);
  maestro.setUpdatePeriod(20); // speed up maestro updates.

  // reverses forward direction of left tankdrive wheels.
  //  chassis.reverseMotor(3,true);
  chassis.reverseRightMotors(true);
  //shooter_servo.setPosition(shooter_pos);
}

void ledService()
{
  // cant add any led services right now as pin 13 is being used for driving a pololuG2.
}

void loop() {
  // trigger network services
  myPacketSerial.update();

  // trigger led (human readable) services.
  ledService();

  // drive chassis
  

  // update target values every 9ms
  if (millis() - last_update > 9)
  {

    // Button Variables
    bool cross = ps2x.Button(PSB_CROSS);
    bool circle = ps2x.Button(PSB_CIRCLE);
    bool triangle = ps2x.Button(PSB_TRIANGLE);
    bool square = ps2x.Button(PSB_SQUARE);
    bool L1 = ps2x.Button(PSB_L1);
    bool L2 = ps2x.Button(PSB_L2);
    bool R1 = ps2x.Button(PSB_R1);
    bool R2 = ps2x.Button(PSB_R2);
    bool Triangle = ps2x.Button(PSB_TRIANGLE);
    bool Square = ps2x.Button(PSB_SQUARE);
    bool Cross = ps2x.Button(PSB_CROSS);
    bool Circle = ps2x.Button(PSB_CIRCLE);
    bool PAD_Up = ps2x.Button(PSB_PAD_UP);
    bool PAD_Down = ps2x.Button(PSB_PAD_DOWN);
    bool R1_Pressed = ps2x.ButtonPressed(PSB_R1);
    bool R1_Released = ps2x.ButtonReleased(PSB_R1);
    bool L1_Pressed = ps2x.ButtonPressed(PSB_L1);
    bool L1_Released = ps2x.ButtonReleased(PSB_L1);
    bool R2_Pressed = ps2x.ButtonPressed(PSB_R2);
    bool R2_Released = ps2x.ButtonReleased(PSB_R2);
    bool L2_Pressed = ps2x.ButtonPressed(PSB_L2);
    bool L2_Released = ps2x.ButtonReleased(PSB_L2);

    Vec2 vec = Vec2(ps2x.JoyStick(PSS_LX), -ps2x.JoyStick(PSS_LY));
    pss_rx_values[pss_rx_iterator] = -ps2x.JoyStick(PSS_RX);
    pss_rx_iterator++;
    if(pss_rx_iterator > 2)
    {
      pss_rx_iterator = 0;
    }
    unsigned char zero_count;
    for(unsigned char i = 0; i < 3; i++)
    {
      if(pss_rx_values[i] == 0)
      {
        zero_count++;
      }
    }
    if(!Cross && !Square && !Triangle && !Circle)
    {
      //
      if(zero_count >= 2)
      {
        chassis.drive(Vec2::angle(vec), Vec2::magnitude(vec), 0);
      } else {
        chassis.drive(Vec2::angle(vec), Vec2::magnitude(vec), -ps2x.JoyStick(PSS_RX));
      }
    } else if (Triangle) {
      chassis.drive(1.57, 0.5, 0);
    } else if (Square) {
      chassis.drive(3.14, 0.5, 0);
    } else if (Circle) {
      chassis.drive(0, 0.5, 0);
    } else if (Cross) {
      chassis.drive(4.71, 0.5, 0);
    }
    

    // shooter servo
    if(PAD_Up)
    {
      shooter_pos += 0.25;
    }
    if(PAD_Down)
    {
      shooter_pos -= 0.25;
    }
    shooter_pos = constrain(shooter_pos, 55, 135);
    shooter_servo.setPosition(shooter_pos);
    //Serial.println(readBatteryMillivoltsSV());
    // Intake
    // actuate devices to intake tennis balls. Arguments are experimetnally determined / calculated.
    if (R1_Pressed)
    {
      intakemotor.setPower(-1); // all speed controllers extended from Motor class have setPower(float power) function. Value between -1 (FUll Reverse) to 1 (FULL Forward).
      chamber.setPosition(170); // HS485 servos have setPosition(float position). Value between 0 and 180 degrees.
      intake.setPosition(90.7);
    }

    // return to idle positions
    if (R1_Released)
    {
      intakemotor.setPower(0);
      chamber.setPosition(128);
      intake.setPosition(178);
    }

    // Outake
    if (L1_Pressed)
    {
      intakemotor.setPower(1);
      chamber.setPosition(170);
      intake.setPosition(90.7);
    }

    if (L1_Released)
    {
      intakemotor.setPower(0);
      chamber.setPosition(128);
      intake.setPosition(178);
    }

    // Actuate Chamber (put tennis ball into shooter)
    if (R2_Pressed)
    {
      chamber.setPosition(30);
    }

    if (R2_Released)
    {
      chamber.setPosition(128);
    }

    //Arm Shooter
    if (L2)
    {
      shooter.setPower(-1);
    } else {
      shooter.setPower(0);
    }

    ps2x.read_gamepad(); // clear release&pressed flags.
    last_update = millis();

    PololuG2::iterate();
  }

  if (network.getLastPS2PacketTime() > 500)
  {
    //maestro.queTarget(3, 0);
    digitalWrite(2, LOW);
    maestro.queTarget(6, 0);
    maestro.queTarget(9, 0);
    maestro.queTarget(12, 0);
    maestro.queTarget(15, 0);

    shooter.setPower(0);
  } else {
    digitalWrite(2, HIGH);
    //maestro.queTarget(3, 7000);
    maestro.queTarget(6, 7000);
    maestro.queTarget(9, 7000);
    maestro.queTarget(12, 7000);
    maestro.queTarget(15, 7000);
    //maestro.queTarget(3, 7000);

  }

  // Trigger MiniMaestroService
  maestro.service();
}

