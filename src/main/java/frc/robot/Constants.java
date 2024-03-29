// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Battery Constants
  public static final double minVoltageYellow = 11.0;
  public static final double minVoltageRed = 12.0;
  public static final double minVoltageRedDouble = 10.0;
  public static final int timeInSecondsSimTest = 5;
  public static final double timeInSecondsGeneralYellow = 360.0;
  public static final double timeInSecondsGeneralRed = 600.0; // Changed for testing
  public static final double timeInSecondsHighCurrentYellow = 120.0;
  public static final double timeInSecondsHighCurrentRed = 180.0;
  public static final double highBatteryCurrentThreshold = 10; // TODO: Put a real value here

  public enum AimModes {
    AUTO,
    // TODO: Plug real values in for these aimModes
    LOW(1500.0, 0.0) {
      @Override
      public AimModes previous() {
        return values()[values().length - 1];
      }
    },
    EJECT(1500.0, 0.0),
    LAUNCH(0.0, 0.0),
    TARMAC(1400.0, 0.0),
    ATARMAC(1400.0, 0.0),
    TEST {
      @Override
      public AimModes next() {
        return values()[0];
      }
    };

    private final double RPM;
    private final double angle;
    private final double distance;

    AimModes() {
      // Used for auto mode ONLY
      this.RPM = 0.0;
      this.angle = 0.0;
      this.distance = 0.0;
    }

    AimModes(double rpm, double a) {
      this.RPM = rpm;
      this.angle = a;
      this.distance = 0.0;
    }

    AimModes(double dist) {
      this.RPM = 0.0;
      this.angle = 0.0;
      this.distance = dist;
    }

    public final double getRPM() {
      return RPM;
    }

    public final double getAngle() {
      return angle;
    }

    public final double getDistance() {
      return distance;
    }

    public AimModes next() {
      return values()[ordinal() + 1];
    }

    public AimModes previous() {
      return values()[ordinal() - 1];
    }
  }

  // TODO: when true, shooter is over shooting
  public static final boolean DebugMode = false; // TODO: change to false for competition time

  // motor controller constants
  public static final int defaultCurrentLimit = 40;

  // DRIVEBASE Constants

  // Constants for wheel motors
  public static final double wheelRadius = 2; // radius of wheel, use for calculating angular values
  // Rate at which the motors reach maximum speed; TODO: tune for optimal performance
  public static final double openLoopRampRate = 0.2;
  public static final double gearRatio = 6.2; // 10.75 : 1 gear ratio <--- kitbot
  // 10.75 motor rotations : 1 wheel rotation
  public static final double inchesInMeter = 39.3701;

  // Actual IDs on robot, used to activate the right motors

  public static final int driveLeftFront = 13; // 13 on real robot, 1 on kitbot
  public static final int driveLeftRear = 14; // 14 on real robot, 2 on kitbot
  public static final int driveRightFront = 6; // 6 on real robot,  3 on kitbot
  public static final int driveRightRear = 7; // 7 on real robot,  4 on kitbot

  // This is used for organizational purposes (Note numbers 0-3 to distinguish between the 4 motors)
  public static final int driveLeftFrontIndex = 0;
  public static final int driveLeftRearIndex = 1;
  public static final int driveRightFrontIndex = 2;
  public static final int driveRightRearIndex = 3;

  public static final int driveBaseCurrentLimit = 50;
  public static final double driveBaseTurnRate = 0.85;

  // drive base pid values
  public static final double[] driveRightPID = {0.00045, 0.00000085, 0};
  public static final double[] driveLeftPID = {0.00045, 0.0000025, 0};

  // AUTONOMOUS Constants

  public enum Auton {
    // spotless:off
    PUSHTAXI("Push Taxi", "paths/TaxiOutPushBall.wpilib.json"),
    
    INTAKETAXI("Intake Taxi", "paths/TaxiOutGrabBall.wpilib.json"),

    ONEBALL("One Ball", "paths/OneBall.wpilib.json"),

    TWOBALL("Two Ball", "paths/TwoBall1.wpilib.json", 
                        "paths/TwoBall2.wpilib.json"),

    TWOBALLSTEAL1("Two Ball Steal 1", "paths/TwoBall1.wpilib.json", 
                                      "paths/TwoBall2.wpilib.json",
                                      "paths/TwoBallSteal1.wpilib.json",
                                      "paths/TwoBallSteal1Pt2.wpilib.json"),

    TWOBALLSTEAL2("Two Ball Steal 2", "paths/TwoBall1.wpilib.json", 
                                      "paths/TwoBall2.wpilib.json",
                                      "paths/TwoBallSteal2.wpilib.json"),

    THREEBALL("Three Ball", "paths/Three1.wpilib.json", 
                            "paths/Three2.wpilib.json", 
                            "paths/Three3.wpilib.json", 
                            "paths/Three4.wpilib.json"),

    FOURBALL("Four Ball", "paths/Four1.wpilib.json", 
                          "paths/Four2.wpilib.json", 
                          "paths/Four3.wpilib.json", 
                          "paths/Four4.wpilib.json"),

    FIVEBALL("Five Ball", "paths/Five1.wpilib.json", 
                          "paths/Five2.wpilib.json", 
                          "paths/Five3.wpilib.json", 
                          "paths/Five4.wpilib.json", 
                          "paths/Five5.wpilib.json", 
                          "paths/Five6.wpilib.json"),

    TEST("Test", THREEBALL);
    // change according to what mode you want to test
    // spotless:on

    private String paths[];
    private String name;

    private Auton(String name, String... paths) {
      this.name = name;
      this.paths = paths;
    }

    // for the TEST constructor
    private Auton(String name, Auton a) {
      this.name = name;
      this.paths = a.getPaths();
    }

    public String getName() {
      return this.name;
    }

    public String[] getPaths() {
      return this.paths;
    }
  }

  // Volts, constants for ramseteCommand
  public static final double ksVolts = 0.26514; // Ks
  public static final double kvVoltSecondsPerMeter = 0.81633; // Kv, Velocity
  public static final double kaVoltSecondsSquaredPerMeter = 0.15052; // Ka, Accelleration

  public static final double trackWidth = 0.559; // track width of kitbot
  public static final DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(trackWidth);

  public static final double arbFeedForward =
      2.5778E-07; // voltage applied to the motor after the result of the specified control

  // Pathweaver constants, baselind values, units: meters per second
  public static final double ramseteB = 2; // Convergence, larger values are more aggressive
  public static final double ramseteZeta = 0.7; // Damping, larger values offer more damping

  public static final double defaultInitialWaitTime = 0;

  // Encoder constants

  // Controller modes
  public static final boolean oneController = true;

  // Encoder Constants
  // TODO: Change to true when using external encoders
  public static final boolean usingExternal = false;
  public static final int encoderCountsPerRev = 8192;

  // Intake Constants
  public static final int intakeMotorOneID = 1;
  public static final int intakeDeployMotorID = 8;
  public static final double[] intakeDeployPID = {0.25, 0, 1};
  public static final double intakeDeployPos = 72;
  public static final double intakeRetractPos = 5;
  public static final int intakeDeployCurrent = 30;

  public static final double intakeMotorSpeed = 1.0;
  public static final int initialBallSensorChannel = 0;
  public static final int middleBallSensorChannel = 1;
  public static final int finalBallSensorChannel = 2;
  public static final int sensorsDownLimit = 3;

  // CDS Constants
  public static final boolean ballManagementEnabled =
      true; // TODO: turning it on messes with shooter on the test bot

  public static final int CDSBeltID = 3;
  public static final int CDSWheelControllerOneID = 2;
  public static final int CDSWheelControllerTwoID = 9;
  public static final double CDSBeltSpeed = .75;
  public static final double CDSAutoBeltSpeed = .75;
  public static final double CDSWheelControllerSpeed = 0.80;
  public static final int frontSensorActivation = 200;
  public static final int middleSensorActivation = 450;
  public static final int backSensorActivation = 600;

  public static final boolean testMode = false; // if false CDS will eject balls of wrong color

  public static final double reverseStopperWheelSpeed = -0.125;

  // spotless:off
  // Controller Constants {
  public static final int portNumber0 = 0;
  public static final int portNumber1 = 1;

  // Buttons not in use
  public static final int XButton = 1;
  public static final int AButton = 2;
  public static final int BButton = 3;
  public static final int YButton = 4;

  // Intake Subsystem
  public static final int LBumper = 5; // Intake forward
  public static final int RBumper = 6; // Intake reverse

  // CDS Subsystem
  public static final int LTriggerButton = 7; // CDS forward
  public static final int RTriggerButton = 8; // CDS reverse

  // Shooter Prime/LimeLight
  public static final int backButton = 9; // Button starts ShooterPrime
  public static final int startButton = 10; // Button to align LimeLight

  // Shooter Subsystem mode change
  public static final int LJoystickButton = 11; // Button for Shooter mode
  public static final int RJoystickButton = 12; // BUtton for Shooter mode

  // POV's not in use
  public static final int POVup = 0;
  public static final int POVdown = 180;
  public static final int POVright = 90;
  public static final int POVleft = 270;

  // Joystick

  // Unused but will easily be accidentally activated if used
  public static final int leftJoystickX = 0;
  public static final int leftJoystickY = 1; // arcade forward / tank left turning
  public static final int rightJoystickX = 2; // arcade turning
  public static final int rightJoystickY = 3; // tank right turning

  // Shooter Constants
  public static final class Shooter {
    // Motor IDs
    public static final int shooterID = 10; // ID of the shooter
    public static final int shooter2ID = 11; // ID of the second shooter motor
    // public static final int hoodID = 0; // ID of the hood;
    public static final int shooterCargoID = 4;

    // LL Placement
    public static final double highHeight =
        8.0 + 8.0 / 12.0; // Height of the high goal in ft from the carpet
    public static final double lowHeight =
        5.0 + 7.75 / 12.0; // Height of the low goal in ft from the carpet
    public static final double LLHeight =
        28.0 / 12.0; // Height of the limelight in ft from the carpet
    public static final double LLAngle =
        54.0; // Angle that the limelight is mounted at from a vertical plane, ensure this is as
    // exact as possible

    // Motor Speeds
    public static final double cargoForward = 1.0;
    public static final double cargoReverse = -0.4;

    // PID settings
    // 2.5e-4, 2.5e-7, 2e-6, 1e-4
    public static final double kPIDFArray[] = {1.4e-09, 3.0E-8, 0};
    // public static final double kPIDFArray[] = {0.000003, 0.00000004, 0};
    // public static final double kPIDFArray[] = {2.5e-8, 5.5e-8, 0}; 7e-9
    public static final double kMaxIAccum = 0.9;
    public static final int kMaxISlot = 0;
    public static final double kMaxOutput = 1.0;
    public static final double kMinOutput = 0;
    public static final double kA = 0.35; // Smoothing alpha, do not cofuse with kAg
    // PID FF gains
    public static final double kSg = 0.56246;
    public static final double kVg = 0.13981;
    public static final double kAg = 0.0087876;
  }

  // Climb Constants

  // mode

  // climb motor ids
  public static final int armMotorOne = 5;
  public static final int armMotorTwo = 12;
  public static final int poleMotorOne = 15;
  public static final int poleMotorTwo = 16;

  public static final int armHeightMax = 132;
  public static final int armHeightMin = 0;
  public static final int armHeightFeather1 = -20;
  public static final int armHeightFeather2 = -35;
  public static final int armFeatherRPM1 = -5000;
  public static final int armFeatherRPM2 = -6000;
  public static final double poleHeightMax = 6;
  public static final int poleHeightMin = -60;
  public static final int poleHeightDeploy = -12;

  public static final double[] armPosPID = {0.125, 0, 0};
  public static final double[] armVelocityPID = {4e-5, 0, 0};
  public static final double[] polePosPID = {0.125, 0, 0};
  public static final int armPosPIDSlot = 1;
  public static final int armVelPIDSlot = 2;

  public static final double climbMaxOutput = 1.0;
  public static final int climbPoleLowCurrent = 15;
  public static final int climbArmLowCurrent = 10;
  public static final int climbPoleHighCurrent = 50;
  public static final int climbArmHighCurrent = 60;

  // servo for hooks
  // TODO: find the real ids as well as make sure the set point is correct
  public static final int climbServoIDOne = 0;
  public static final int climbServoIDTwo = 1;
  public static final double climbServo1Locked = 0.35;
  public static final double climbServo1Unlocked = 1;
  public static final double climbServo2Locked = 0.65;
  public static final double climbServo2Unlocked = 0;

  public static final double climbServoSetPoint = 0.5;

  public static final double driveSpeedHigh = 1;
  public static final double driveSpeedLow = 0.6;

  public static final double controllerDeadZone = 0.1;

  public static final double climbSequence1Timeout = 20.0;

  public static final double armUpSpeed = -1;
  public static final double armDownSpeed = -0.65;
  public static final double poleInSpeed = 0.5;
  public static final double poleOutSpeed = 0.6;

  public static final double armSetIMaxAccum = 0.45;
  public static final double poleSetIMaxAccum = 0.25;

  public static final int climbArmDeadband = 1;
}
