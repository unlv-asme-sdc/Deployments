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

//shooter angles
float shooterMin = 60;
float shooterMax = 180;
float shooterangle = (shooterMin + shooterMax) / 2;


float intakeservo_idle = 805/13 + 20;
float intakeservo_intake = 3815/26;
float intakeservo_initial = 9475/52;
	// Chamber Servo
float chamber_intake = 325/2;
float chamber_idle = 1600/13;
float chamber_shoot = 575/26;
	// Shooter Servo
float shooterservo_Min = 2100/13;
float shooterservo_Max = 3065/52;
//intake angles: Correcintg Calculator Mistake
	// Intake Servos
/*
float intakeservo_idle = 3605/52;
float intakeservo_intake = 7365/52 + 3;
float intakeservo_initial = 2455/52;
	// Chamber Servo
float chamber_intake = 325/2;
float chamber_idle = 1600/13;
float chamber_shoot = 575/26;
	// Shooter Servo
float shooterservo_Min = 2100/13;
float shooterservo_Max = 3065/52;
*/


//

// MiniMaestroServices construction
// 52 recieve, 50 transmit
//SoftwareSerial maestroSerial(52, 50); // Connect A1 to Maestro's RX. A0 must remain disconnected.
#define maestroSerial Serial1

MiniMaestroService maestro(maestroSerial);

// Subsystems construction
HS485 chamber = HS485(maestro, 0); // Some objects can use MiniMaestroService to controll devices. See docs.
HS485 intake = HS485(maestro, 1);
HS485 shooter_servo = HS485(maestro, 3);
TalonSR shooter = TalonSR(maestro, 2);
PololuG2 intakemotor = PololuG2(maestro, 9, 10, 11, true);
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
LSMHeadless gyro;
bool armMotors = true;
bool gyrodrive = false;

float leftvalue = 0;
float rightvalue = 0;
unsigned long last_blink;
unsigned long last_update;

// speed caps
float thrustcap = 1;
float turncap = 1;
void setup() {
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

  // enable debug usb (Serial) and radio serial (Serial1)
  maestroSerial.begin(57600); 
  // binds radio PacketSerial(encoding&decoding services) to NetworkTable class. Uses lambda expressions.
  myPacketSerial.begin(115200);
  myPacketSerial.setPacketHandler([](const uint8_t* buffer, size_t size) {
    network.processPacketFromSender(myPacketSerial, buffer, size);
  });

  // sets ps2x controlls.
  network.setPS2(ps2x);
  maestro.setUpdatePeriod(40); // speed up maestro updates.

  // reverses forward direction of left tankdrive wheels.
  //chassis.reverseRightMotors(true);
	//chassis.reverseMotor(1, true);
	chassis.reverseMotor(2, true);
	chassis.reverseMotor(3,true);
	chassis.reverseMotor(4, true);// Uncomment this for Desperado
	//chassis.reverseMotor(4, true);
  gyro.init();
  gyro.calibrate();

  // prevents devices from actuating on startup.
  delay(1500);
	subsystems.setShooterAngle(shooterMin);
	subsystems.setIntakeAngle(intakeservo_initial);
}

void ledService()
{
  // cant add any led services right now as pin 13 is being used for driving a pololuG2.
}

void loop() {
  // trigger network services
  myPacketSerial.update();
gyro.iterate();

  // trigger led (human readable) services.
  ledService();

  // drive chassis
  

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

    if(gyrodrive)
    {
    	chassis.drive(Vec2::angle(vec) - gyro.getRelativeYaw()*3.14/180, Vec2::magnitude(vec)*thrustcap, -ps2x.JoyStick(PSS_RX)*turncap);
    } else {
	    chassis.drive(Vec2::angle(vec), Vec2::magnitude(vec)*thrustcap, -ps2x.JoyStick(PSS_RX)*turncap);
    }

    if(Cross)
    {
    	gyrodrive = !gyrodrive;
    }
    if(Triangle)
    {
    	gyro.zero();
    }
    if(Square)
    {
    	gyro.trim(1);
    }
    if(Circle)
    {
    	gyro.trim(-1);
    }
    

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
	subsystems.pushSequence(SHOOTER_POWER, 2000, (float)0.0);
	subsystems.pushSequence(CHAMBER_SHOOT, 1500);
	subsystems.pushSequence(SHOOTER_POWER, 0, (float)1.0);
	subsystems.pushSequence(SHOOTER_ANGLE, 0, (float)4645/52);
	subsystems.pushSequence(CHAMBER_IDLE, 0, true);
    }

    //Arm Shooter
    if (L2)
    {
	//subsystems.pushSequence(CHAMBER_IDLE,1500);
	subsystems.pushSequence(CHAMBER_IDLE,0);
	subsystems.pushSequence(SHOOTER_POWER, 2000, (float)0.0);
	subsystems.pushSequence(CHAMBER_SHOOT, 1500);
	subsystems.pushSequence(SHOOTER_POWER, 0, (float)1);
	subsystems.pushSequence(SHOOTER_ANGLE, 0, (float)5685/52);
	subsystems.pushSequence(CHAMBER_IDLE, 0, true);
    }

    ps2x.read_gamepad(); // clear release&pressed flags.
    last_update = millis();

    PololuG2::iterate();

	// Arm Motors
	if(Select_Pressed)
	{
		armMotors = !armMotors;
	}
	if(Start_Pressed)
	{
		if(gyro.getInitialized())
		{
			gyro.calibrate();
			//gyro.init();
			gyro.zero();
			
		} else {
			gyro.init();
			if(gyro.getInitialized())
			{
				gyro.calibrate();
				gyro.zero();
			}
				
		}

	}
  }
	subsystems.iterate();

  if (network.getLastPS2PacketTime() > 500 || !armMotors)
  {
    //maestro.queTarget(3, 0);
    digitalWrite(2, LOW);
    digitalWrite(4, LOW);
    maestro.queTarget(6, 0);
    maestro.queTarget(9, 0);
    maestro.queTarget(12, 0);
    maestro.queTarget(15, 0);
    shooter.setPower(0);
  } else {
    digitalWrite(2, HIGH);
    digitalWrite(4, HIGH);
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

