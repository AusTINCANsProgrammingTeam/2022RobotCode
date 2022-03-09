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

  public enum Subsystems {
    // Change booleans to disable a subsystem in RobotContainer
    // spotless:off
    DriveBaseSubsystem(true),
    CDSSubsystem      (true),
    IntakeSubsystem   (true),
    ShooterSubsystem  (true),
    LimelightSubsystem(true),
    ClimbSubsystem    (true);
    // spotless:on

    private final Boolean enabled;

    Subsystems(Boolean b) {
      this.enabled = b;
    }

    public final Boolean isEnabled() {
      return this.enabled;
    }
  }

  public enum AimModes {
    AUTO,
    // TODO: Plug real values in for these aimModes
    LOW(3200.0, 0.0) {
      @Override
      public AimModes previous() {
        return values()[values().length - 1];
      }
    },
    EJECT(2500.0, 0.0),
    LAUNCH(0.0, 0.0),
    TARMAC(4200.0, 0.0),
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

  // motor controller constants
  public static final int defaultCurrentLimit = 40;

  // DRIVEBASE Constants

  // Constants for wheel motors
  public static final double wheelRadius = 2; // radius of wheel, use for calculating angular values
  public static final double openLoopRampRate =
      0.2; // Rate at which the motors reach maximum speed; TODO: tune for optimal performance
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

  public static final int driveBaseCurrentLimit = 60;

  // drive base pid values
  public static final double[] driveRightPID = {
    0.00035, 0.0000008, 0
  }; // TODO: need to tune for real robot
  public static final double[] driveLeftPID = {0.000005, 0.0000008, 0};

  // AUTONOMOUS Constants

  // Path json files
  public static final String taxiPath = "paths/TaxiOut.wpilib.json";
  public static final String oneBallPath = "paths/TaxiOutFromFender.wpilib.json";
  public static final String twoBallPath = "paths/TaxiOutToGrabBall.wpilib.json";

  // Volts, constants for ramseteCommand
  public static final double ksVolts = 0.28665; // Ks
  public static final double kvVoltSecondsPerMeter = 1.4563; // Kv, Velocity
  public static final double kaVoltSecondsSquaredPerMeter = 0.21703; // Ka, Accelleration

  public static final double arbFeedForward =
      0.00046254; // voltage applied to the motor after the result of the specified control mode
  public static final double trackWidth = 0.559; // track width of kitbot
  public static final DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(trackWidth);
  public static final double unitsPerRotation = 0.4787787204;

  // Pathweaver constants, baselind values, units: meters per second
  public static final double ramseteB = 2; // Convergence, larger values are more aggressive
  public static final double ramseteZeta = 0.7; // Damping, larger values offer more damping

  public static final double delaytaxi = 1.0; // 1 second wait time
  public static final double delayshot = 0.5; // 0.5 second wait time

  // TODO: Replace 0.69 with actual track width in meters and run characterization on real robot

  // Encoder constants

  // Controller modes
  public static final boolean oneController = true;

  // Encoder Constants
  // TODO: Change to true when using external encoders
  public static final boolean usingExternal = false;
  public static final int encoderCountsPerRev = 8192;

  // Intake Contstants
  public static final int intakeMotorOneID = 1;
  public static final double intakeMotorSpeed = 0.70;
  public static final int initialBallSensorChannel = 0;
  public static final int middleBallSensorChannel = 1;
  public static final int finalBallSensorChannel = 2;

  // CDS Constants
  public static final int CDSBeltID = 3;
  public static final int CDSWheelControllerOneID = 2;
  public static final int CDSWheelControllerTwoID = 9;
  public static final double CDSBeltSpeed = 0.40;
  public static final double CDSWheelControllerSpeed = 0.25;
  public static final int frontSensorActivation = 200;
  public static final int middleSensorActivation = 450;
  public static final int backSensorActivation = 600;

  public static final boolean testMode = false; // if false CDS will eject balls of wrong color

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
    public static final int shooterID = 10; // ID of the shooter
    public static final int shooter2ID = 11; // ID of the second shooter motor
    // public static final int hoodID = 0; // ID of the hood;
    public static final int shooterCargoID = 4;

    public static final double highHeight =
        8.0 + 8.0 / 12.0; // Height of the high goal in ft from the carpet
    public static final double lowHeight =
        5.0 + 7.75 / 12.0; // Height of the low goal in ft from the carpet
    public static final double LLHeight =
        28.0 / 12.0; // Height of the limelight in ft from the carpet
    public static final double LLAngle =
        54.0; // Angle that the limelight is mounted at from a vertical plane, ensure this is as
    // exact as possible
    public static final double cargoForward = 0.65;
    public static final double cargoReverse = -0.4;
    public static final double kP = 2.5e-4;
    public static final double kI = 19e-6;
    public static final double kD = 0.005;
    public static final double kF = 0.0;
    public static final double kIZone = 0.9;
    public static final double kMaxOutput = 0;
    public static final double kMaxI = 0.9;
    public static final double kMaxISlotId = 0;
    public static final double kMinOutput = 1;
    public static final double kA = 0.075;
  }

  // Climb Constants
  public static final int ClimbMotorOne = 5;
  public static final int ClimbMotorTwo = 12;
  public static final int climbHeightMax = 65;
  public static final double[] climbRightPID = {0.25, 0.005, 1.0};
  public static final double[] climbLeftPID = {0.25, 0.005, 1.0};
  // public static final int LimitSwitchChannel = 12; // Check what number this needs to be
}
