
// Wireless Fix for Renegade Robot
// Renegade Constants
#include <SoftwareSerial.h>
#include <MiniMaestroService.h>
#include <TalonSR.h>
#include <PololuG2.h>
#include <HolonomicDrive.h>
#include <PS2X_lib.h>
#include <HS485.h>
#include <NetworkTable.h>
//#include <AStar32U4.h>
#include <LSMHeadless.h>
#include <OSubsystems.h>
// DELETE MEEEE
float shooterpower = 0;
float shooterChamberDelay = 750;
float shooterPostChamberDelay = 750;
/*
//shooter angles Desperado
float shooterMin = 60;
float shooterMax = 180;
float shooterangle = (shooterMin + shooterMax) / 2;


float intakeservo_idle = 10.3;
float intakeservo_intake = 81.13;
float intakeservo_initial = intakeservo_idle;
	// Chamber Servo
float chamber_intake = 177;
float chamber_idle = 156.3;
float chamber_shoot = 65.6;
	// Shooter Servo
float shooterservo_Min = 146;
float shooterservo_Max = 59.7;
*/
//shooter angles Renegade
float shooterMin = 62.69;
float shooterMax = 144.5;
float shooterangle = (shooterMin + shooterMax) / 2;


float intakeservo_idle = 77.4;
float intakeservo_intake = 153;//146.2
float intakeservo_initial = 77.4;
	// Chamber Servo
float chamber_intake = 166.6;
float chamber_idle = 134;
float chamber_shoot = 31.3;
	// Shooter Servo
//
// MiniMaestroServices construction
// 3 recieve, 5 transmit
SoftwareSerial maestroSerial(3, 5); // Connect A1 to Maestro's RX. A0 must remain disconnected.
MiniMaestroService maestro(maestroSerial);

// Subsystems construction
HS485 chamber = HS485(maestro, 0); // Some objects can use MiniMaestroService to controll devices. See docs.
HS485 intake = HS485(maestro, 1);
HS485 shooter_servo = HS485(maestro, 3);
//Renegade Only
//TalonSR shooter = TalonSR(maestro, 2);
//Desperado
PololuG2 shooter = PololuG2(maestro, 6, 7, 8, true); PololuG2 intakemotor = PololuG2(maestro, 9, 10, 11, true);
// Drive base construction
PololuG2 motor1 = PololuG2(maestro, 12, 13, 14, true); 
PololuG2 motor2 = PololuG2(2, 9, 7, true);
PololuG2 motor3 = PololuG2(4, 10, 8, true);
PololuG2 motor4 = PololuG2(maestro, 15, 16, 17, true);
// Can construct drive bases using any speed controllers extended from Motor class. (TalonSR and PololuG2).
HolonomicDrive chassis = HolonomicDrive(motor1, motor2, motor3, motor4);

OSubsystems subsystems = OSubsystems(chassis, shooter, shooter_servo, chamber, intakemotor, intake);

// Network & Controller construction
PS2X ps2x;
NetworkTable network = NetworkTable(10, 10);
PacketSerial myPacketSerial;

// LSM Devices (GYRO)
bool armMotors = true;

float leftvalue = 0;
float rightvalue = 0;
unsigned long last_blink;
bool blink_bool = false;
unsigned long last_update;

// speed caps
float thrustcap = 1;
float turncap = 1;
void setup() {
  Serial1.begin(115200);
  myPacketSerial.setStream(&Serial1);
  myPacketSerial.setPacketHandler([](const uint8_t* buffer, size_t size) {
    network.processPacketFromSender(myPacketSerial, buffer, size);
  });
  maestroSerial.begin(57600); // 9600 
  // binds radio PacketSerial(encoding&decoding services) to NetworkTable class. Uses lambda expressions.
	pinMode(13, OUTPUT);
	subsystems.chamber_intake_pos = chamber_intake;
	subsystems.chamber_shoot_pos = chamber_shoot;
	subsystems.chamber_idle_pos = chamber_idle;
	subsystems.intake_idle_pos = intakeservo_idle;
	subsystems.intake_intake_pos = intakeservo_intake;
	subsystems.intake_roller_in = -1;
	subsystems.intake_roller_out = 1;
  // prevents devices from actuating on startup.
  delay(1000);
  // initialize MiniMaestroService communication

  // enable ps2x networking
  ps2x.config_gamepad();
  ps2x.read_gamepad();


  // sets ps2x controlls.
  network.setPS2(ps2x);
  maestro.setUpdatePeriod(100); // speed up maestro updates.

  // reverses forward direction of left tankdrive wheels.
	// Renegade
	//chassis.reverseMotor(1, true);
	//chassis.reverseMotor(2, true);
	//chassis.reverseMotor(3,true);
	chassis.reverseMotor(4, true);// Uncomment this for Desperado
/*
	// Desperado
	chassis.reverseMotor(1, true);
	//chassis.reverseMotor(2, true);
	//chassis.reverseMotor(3,true);
	//chassis.reverseMotor(4, true);// Uncomment this for Desperado
	//chassis.reverseMotor(4, true);
*/

  // prevents devices from actuating on startup.
  delay(1500);
	subsystems.setShooterAngle(shooterMin);
	subsystems.setIntakeAngle(intakeservo_initial);
}

void ledService()
{
  // cant add any led services right now as pin 13 is being used for driving a pololuG2.
	unsigned long timer = millis();
	if((timer - last_blink) > 500)
	{
		network.sendBuffer(&myPacketSerial, 6);
		blink_bool = !blink_bool;
		last_blink = timer;
		digitalWrite(13, blink_bool);

	}
}

void loop() {
  ledService();
  myPacketSerial.update();

  // update target values every 9ms
  if (millis() - last_update > 9)
  {

    // Button Variables
    bool L1 = ps2x.Button(PSB_L1);
    bool L2 = ps2x.Button(PSB_L2);
    bool R1 = ps2x.Button(PSB_R1);
    bool R2 = ps2x.Button(PSB_R2);
    bool Triangle = ps2x.ButtonPressed(PSB_TRIANGLE);
    bool Square = ps2x.ButtonPressed(PSB_SQUARE);
    bool Cross = ps2x.ButtonPressed(PSB_CROSS);
    bool Circle = ps2x.ButtonPressed(PSB_CIRCLE);
    bool PAD_Up = ps2x.ButtonPressed(PSB_PAD_UP);
    bool PAD_Down = ps2x.ButtonPressed(PSB_PAD_DOWN);
    bool PAD_Right = ps2x.ButtonPressed(PSB_PAD_RIGHT);
    bool PAD_Left = ps2x.ButtonPressed(PSB_PAD_LEFT);
    bool R1_Pressed = ps2x.ButtonPressed(PSB_R1);
    bool R1_Released = ps2x.ButtonReleased(PSB_R1);
    bool L1_Pressed = ps2x.ButtonPressed(PSB_L1);
    bool L1_Released = ps2x.ButtonReleased(PSB_L1);
    bool R2_Pressed = ps2x.ButtonPressed(PSB_R2);
    bool R2_Released = ps2x.ButtonReleased(PSB_R2);
    bool L2_Pressed = ps2x.ButtonPressed(PSB_L2);
    bool L2_Released = ps2x.ButtonReleased(PSB_L2);
    bool Select_Pressed = ps2x.ButtonPressed(PSB_SELECT);
    bool Start_Pressed = ps2x.ButtonPressed(PSB_START);

	if(PAD_Up)
	{
		shooterpower += .03;
	}
	if(PAD_Down)
	{
		shooterpower-= .03;
	}
	if(PAD_Right)
	{
		shooterangle -= 10; 
	}
	if(PAD_Left)
	{
		shooterangle += 10; 
	}
	shooterangle = constrain(shooterangle, shooterMin, shooterMax);
	subsystems.setShooter(shooterpower);
	subsystems.setShooterAngle(shooterangle);

    Vec2 vec = Vec2(ps2x.JoyStick(PSS_LX), -ps2x.JoyStick(PSS_LY));
chassis.drive(0, 0.5, 0);

    chassis.drive(Vec2::angle(vec), Vec2::magnitude(vec)*thrustcap, -ps2x.JoyStick(PSS_RX)*turncap);

    // Intake
    // actuate devices to intake tennis balls. Arguments are experimetnally determined / calculated.
    if (R1_Pressed)
    {
	subsystems.setSystemsIntake();
    }

    // return to idle positions
    if (R1_Released)
    {
	subsystems.setSystemsIdle();
    }

    // Outake
    if (L1_Pressed)
    {
	subsystems.setSystemsOuttake();
    }

    if (L1_Released)
    {
	subsystems.setSystemsIdle();
    }

    // Actuate Chamber (put tennis ball into shooter)
    if (R2_Pressed)
    {
	//subsystems.pushSequence(CHAMBER_IDLE,1500);
	subsystems.pushSequence(CHAMBER_IDLE,0);
	subsystems.pushSequence(SHOOTER_POWER, shooterPostChamberDelay, (float)0.0);
	subsystems.pushSequence(CHAMBER_SHOOT, shooterChamberDelay);
	subsystems.pushSequence(SHOOTER_POWER, 0, (float)(1));
	subsystems.pushSequence(SHOOTER_ANGLE, 0, (float)(4645/52));
	subsystems.pushSequence(CHAMBER_IDLE, 0, true);
    }

    //Arm Shooter
    if (L2_Pressed)
    {
	//subsystems.pushSequence(CHAMBER_IDLE,1500);
	subsystems.pushSequence(CHAMBER_IDLE,0);
	subsystems.pushSequence(INTAKE_SERVO_ANGLE, 0, intakeservo_idle);
	subsystems.pushSequence(SHOOTER_POWER, shooterPostChamberDelay, (float)0.0);
	subsystems.pushSequence(INTAKE_SERVO_ANGLE, 0, (float)115);
	subsystems.pushSequence(CHAMBER_SHOOT, shooterChamberDelay);
	subsystems.pushSequence(SHOOTER_POWER, 0, (float)(1));
	subsystems.pushSequence(SHOOTER_ANGLE, 0, (float)(5685/52 + 10));
	subsystems.pushSequence(CHAMBER_IDLE, 0, true);
    }

    ps2x.read_gamepad(); // clear release&pressed flags.

    if(shooter.getTarget() == 0)
    {
	maestro.queTarget(6, 0);
        shooter.setVelocity(0.33);
    } else {
	maestro.queTarget(6, 7000);
	shooter.setVelocity(100);
    }

	// Arm Motors
	if(Select_Pressed)
	{
		armMotors = !armMotors;
	}
    last_update = millis();
    PololuG2::iterate();
  }
	subsystems.iterate();

  if (network.getLastPS2PacketTime() > 500 || !armMotors)
  {
    //maestro.queTarget(3, 0);
    digitalWrite(2, LOW);
    digitalWrite(4, LOW);
    maestro.queTarget(9, 0);
    maestro.queTarget(12, 0);
    maestro.queTarget(15, 0);
    shooter.setPower(0);
  } else {
    digitalWrite(2, HIGH);
    digitalWrite(4, HIGH);
    //maestro.queTarget(3, 7000);
    maestro.queTarget(9, 7000);
    maestro.queTarget(12, 7000);
    maestro.queTarget(15, 7000);
    //maestro.queTarget(3, 7000);
  }

  // Trigger MiniMaestroService
  maestro.service();
}

