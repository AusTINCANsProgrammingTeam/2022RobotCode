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
  
    // TODO: Swap placeholder motor IDs & Button Ports and Numbers for real ones 
    // Constants for wheel motors
  
    // Actual IDs on robot, used to activate the right motors
    // TODO: Values to be changed, these are placeholder values for now
    // Might be reversed
    public static final int kDriveRightFront = 1;
    public static final int kDriveRightRear = 2;
    public static final int kDriveLeftFront = 3;
    public static final int kDriveLeftRear = 4;
    
    // This is used for organizational purposes (Note numbers 0-3 to distinguish between the 4 motors)
    public static final  int kDriveLeftFrontIndex = 0;
    public static final int kDriveLeftRearIndex = 1;
    public static final int kDriveRightFrontIndex = 2;
    public static final int kDriveRightRearIndex = 3;

    public static final int kDriveBaseCurrentLimit = 40;

    // Encoder constants
    // TODO: Replace these with the correct ports
    public static final int kLeftEncoderDIOone = 0;
    public static final int kLeftEncoderDIOtwo = 1;
    public static final int kRightEncoderDIOone = 2;
    public static final int kRightEncoderDIOtwo = 3;
    
    // Pathweaver constants
    public static final double kRamseteB = 2; //Convergence, larger values are more aggressive
    public static final double kRamseteZeta = 0.7; //Damping, larger values offer more damping


    // TODO: Calibrate robot with correct values - These are just placeholers
    // Note: below comments might not be entirely accurate
    public static final double ksVolts = 0.22;                
    
    // Volts
    public static final double kvVoltSecondsPerMeter = 1.98;       //Velocity
    public static final double kaVoltSecondsSquaredPerMeter = 0.2; //Accelleration
    public static final double kPDriveVel = 8.5;                   //Velocity
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.5);    // Replace 0.5 with track width in meters
    
    //Controller constants
    public static final int kDBJoystickPort = 0;

    public static final int kDBLeftJoystickAxisX = 0;
    public static final int kDBLeftJoystickAxisY = 1;
    public static final int kDBRightJoystickAxisX = 2;
    public static final int kDBRightJoystickAxisY = 3;

    // Intake Contstants 
    public static final int kIntakeMotorOneIndex = 0;
    public static final int kIntakeMotorTwoIndex = 1;
    public static final int kIntakeMotorOneID = 11;
    public static final int kIntakeMotorTwoID = 12;
    public static final double kIntakeMotorSpeed = 0.25;

    //CDS Constants
    public static final int kCDSMotorThreeIndex = 0;
    public static final int kCDSMotorThreeID = 13;
    public static final int kCDSMotorFourID = 14;
    public static final int kCDSMotorFiveID = 15;

    public static final double kCDSWheelSpeed = 0.25;
    public static final double kCDSBeltSpeed = 0.25;

    //Joystick Constants
    public static final int kPortNumber = 0;
    public static final int kJoystickButtonNumberOne = 1;
    public static final int kJoystickButtonNumberTwo = 2;
    public static final int kBButton = 2; // Button for hopper
    public static final int kXButton = 3; // Button for reverse hopper
    public static final int kRightBumperButton = 5; // Button for intake 
    public static final int kLeftBumperButton = 6;  // Button to reverse intake
    public static final int kXbutton = 3; // Button for Shooter
    public static final int kDownbutton = 4; // Button for shooter mode
    public static final int kUpbutton = 5; // Button for shooter mode

    // Preset aim constants
    public static final double kLOWRPM = 0.0; // RPM that the LOW aimMode winds to
    public static final double kLOWAngle = 0.0; // Angle that the LOW aimMode adjusts to
    public static final double kLAUNCHRPM = 0.0; // RPM that the LAUNCH aimMode winds to
    public static final double kLAUNCHAngle = 0.0; // Angle that the LAUNCH aimMode adjusts to
    public static final double kTARMACRPM = 0.0; // RPM that the TARMAC aimMode winds to
    public static final double kTARMACAngle = 0.0; // Angle that the TARMAC aimMode adjusts to

    // Shooter Constants
    public static final int kShooterID = 7; // ID of the shooter
    public static final int kHoodID = 8; // ID of the hood;
    public static final double kShooterHeight = 3;
    public static final double kHighHeight = 8.0 + 8.0/12.0; // Height of the high goal in ft from the carpet
    public static final double kLowHeight = 5.0 + 7.75/12.0; // Height of the low goal in ft from the carpet
    public static final double kAirboneTime = 1.2;
    public static final double kGravity = 32;
    public static final double kLLHeight = 10.0/12.0; // Height of the limelight in ft from the carpet
    public static final double kLLAngle = 29.0; // Angle that the limelight is mounted at
    public static final double kGearRatioIn = 34;
    public static final double kGearRatioOut = 14;
    public static final double kGearDiameter = 4; // Gear diameter in inches
    public static final double kBallFlywheelratio = 2;
    public static final int kShooterCargoID = 9;
    public static final double kCargoRotation = 3;
}
