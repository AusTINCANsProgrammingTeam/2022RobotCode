// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.I2C.Port;

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
    ;
  }

  // DRIVEBASE Constants

  // Constants for wheel motors
  public static final double wheelRadius =
      3.0125; // radius of wheel, use for calculating angular values
  public static final double gearRatio = 10.75; // 10.75 : 1 gear ratio <--- kitbot
  // 10.75 motor rotations : 1 wheel rotation
  public static final double inchesInMeter = 39.3701;

  // Actual IDs on robot, used to activate the right motors
  public static final int driveLeftFront = 1;
  public static final int driveLeftRear = 2;
  public static final int driveRightFront = 7;
  public static final int driveRightRear = 8;

  // This is used for organizational purposes (Note numbers 0-3 to distinguish between the 4 motors)
  public static final int driveLeftFrontIndex = 0;
  public static final int driveLeftRearIndex = 1;
  public static final int driveRightFrontIndex = 2;
  public static final int driveRightRearIndex = 3;
  public static final int driveBaseCurrentLimit = 40;

  // drive base pid values
  public static final double[] driveRightPID = {0.00035, 0.0000008, 0};
  public static final double[] driveLeftPID = {0.000005, 0.0000008, 0};

  // AUTONOMOUS Constants

  // Volts, constants for ramseteCommand
  public static final double ksVolts = 0.13323; // Ks,
  public static final double kvVoltSecondsPerMeter = 2.8295; // Kv, Velocity
  public static final double kaVoltSecondsSquaredPerMeter = 0.31462; // Ka, Accelleration

  public static final double kpDriveVel = 2.1938; // Kp, Velocity
  public static final double trackWidth = 0.69;
  public static final DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(trackWidth);
  public static final double unitsPerRotation = 0.4787787204;

  // Pathweaver constants, baselind values, units: meters per second
  public static final double ramseteB = 2; // Convergence, larger values are more aggressive
  public static final double ramseteZeta = 0.7; // Damping, larger values offer more damping

  // TODO: Replace 0.69 with actual track width in meters and run characterization on real robot

  // Encoder constants
  // TODO: Replace these with the correct ports
  public static final int leftEncoderDIOone = 0;
  public static final int leftEncoderDIOtwo = 1;
  public static final int rightEncoderDIOone = 2;
  public static final int rightEncoderDIOtwo = 3;

  // Intake Contstants
  public static final int intakeMotorOneID = 3;
  public static final double intakeMotorSpeed = 0.70;
  public static final int initialBallSensorChannel = 0;
  public static final int middleBallSensorChannel = 1;
  public static final int finalBallSensorChannel = 2;

  // CDS Constants
  public static final int CDSBeltID = 6;
    public static final int CDSWheelControllerOneID = 11;   
    public static final int CDSWheelControllerTwoID = 12;   
  public static final double CDSBeltSpeed = 0.25;
  public static final double CDSWheelControllerSpeed = 0.15;
  public static final Port colorSensorPort = Port.kOnboard; // Placeholder Value, to be changed
  public static final boolean competition = false; // false for testing, true for competition

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

  // DriveBase Subsystem
  public static final int leftJoystickX =
      0; // Unused but will easily be acidentaly activated if used
  public static final int leftJoystickY = 1; // arcade forward / tank left turning
  public static final int rightJoystickX = 2; // arcade turning
  public static final int rightJoystickY = 3; // tank right turning
  // }

  
    // Preset aim constants

    public static final class Shooter{
    public static final double LOWRPM = 0.0; // RPM that the LOW aimMode winds to
    public static final double LOWAngle = 0.0; // Angle that the LOW aimMode adjusts to
    public static final double LAUNCHRPM = 0.0; // RPM that the LAUNCH aimMode winds to
    public static final double LAUNCHAngle = 0.0; // Angle that the LAUNCH aimMode adjusts to
    public static final double TARMACRPM = 0.0; // RPM that the TARMAC aimMode winds to
    public static final double TARMACAngle = 0.0; // Angle that the TARMAC aimMode adjusts to
    //Distance calculation constants
    
    // Shooter Constants
    public static final int shooterID = 4; // ID of the shooter
        //public static final int hoodID = 0; // ID of the hood;
    public static final double shooterHeight = 3;
  public static final double highHeight =
      8.0 + 8.0 / 12.0; // Height of the high goal in ft from the carpet
  public static final double lowHeight =
      5.0 + 7.75 / 12.0; // Height of the low goal in ft from the carpet
    public static final double airboneTime = 1.2;
    public static final double gravity = 32;
    public static final double LLHeight = 8.5/12.0; // Height of the limelight in ft from the carpet
  public static final double LLAngle =
      29.0; // Angle that the limelight is mounted at from a vertical plane, ensure this is as exact
  // as possible
    public static final double gearRatioIn = 34;
    public static final double gearRatioOut = 14;
    public static final double gearDiameter = 4; // Gear diameter in inches
    public static final double ballFlywheelratio = 2;
    public static final int shooterCargoID = 5;
    public static final double cargoRotation = 3;
        public static final double kP =6e-4;
        public static final double kI = 6e-7;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        public static final double kIZone = 0.9;
        public static final double kMaxOutput = 0;
        public static final double kMaxI = 0.9;
        public static final double kMaxISlotId = 0;
        public static final double kMinOutput =1;

    
    }

    //Climb Constants
    public static final int ClimbMotorOne = 9;
    public static final int ClimbMotorTwo = 10;
  public static final int LimitSwitchChannel = 12; // Check what number this needs to be
    
}
