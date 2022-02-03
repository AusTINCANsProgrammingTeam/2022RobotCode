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
    //Distance calculation constants
    public static final double goalHeight = 8.8; //Height of the goal in ft from the carpet

    // TODO: Swap placeholder motor IDs & Button Ports and Numbers for real ones
    // Constants for wheel motors
    // Actual IDs on robot, used to activate the right motors
    // TODO: Values to be changed, these are placeholder values for now
    // Might be reversed

    public static final int driveRightFront = 1;
    public static final int driveRightRear = 2;
    public static final int driveLeftFront = 3;
    public static final int driveLeftRear = 4;

    // This is used for organizational purposes (Note numbers 0-3 to distinguish between the 4 motors)
    public static final int driveLeftFrontIndex = 0;
    public static final int driveLeftRearIndex = 1;
    public static final int driveRightFrontIndex = 2;
    public static final int driveRightRearIndex = 3;
    public static final int driveBaseCurrentLimit = 40;

    // Encoder constants
    // TODO: Replace these with the correct ports
    public static final int leftEncoderDIOone = 0;
    public static final int leftEncoderDIOtwo = 1;
    public static final int rightEncoderDIOone = 2;
    public static final int rightEncoderDIOtwo = 3;

    // Pathweaver constants
    public static final double ramseteB = 2; //Convergence, larger values are more aggressive
    public static final double ramseteZeta = 0.7; //Damping, larger values offer more damping

    // TODO: Calibrate robot with correct values - These are just placeholers

    // Note: below comments might not be entirely accurate
    public static final double sVolts = 0.22;

    // Volts
    public static final double vVoltSecondsPerMeter = 1.98;       //Velocity
    public static final double aVoltSecondsSquaredPerMeter = 0.2; //Accelleration
    public static final double PDriveVel = 8.5;                   //Velocity
    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(0.5);    // Replace 0.5 with track width in meters

    //Controller constants
    public static final int DBJoystickPort = 0;
    public static final int DBLeftJoystickAxisX = 0;
    public static final int DBLeftJoystickAxisY = 1;
    public static final int DBRightJoystickAxisX = 2;
    public static final int DBRightJoystickAxisY = 3;

    // Intake Contstants
    public static final int intakeMotorOneID = 11;
    public static final int intakeMotorTwoID = 12;
    public static final int intakeWheelOneID = 3;
    public static final int intakeWheelTwoID = 4;
    public static final double intakeMotorSpeed = 0.70;
    public static final int initialBallSensorChannel = 0;
    public static final int middleBallSensorChannel = 1;
    public static final int finalBallSensorChannel = 2;

    //CDS Constants
    public static final int CDSBeltID = 13;
    public static final int CDSWheelControllerID = 14;
    public static final double CDSBeltSpeed = 0.15;
    public static final double CDSWheelControllerSpeed = 0.25;

    //Joystick Constants
    public static final int portNumber = 0;
    public static final int joystickButtonNumberOne = 1;
    public static final int joystickButtonNumberTwo = 2;
    public static final int BButton = 2; // Button for hopper
    public static final int YButton = 4; // Button for reverse hopper
    public static final int rightBumperButton = 5; // Button for intake
    public static final int leftBumperButton = 6;  // Button to reverse intake
    public static final int Xbutton = 3; // Button for Shooter
    public static final int downbutton = 4; // Button for shooter mode
    public static final int upbutton = 5; // Button for shooter mode
    public static final int AButton = 2; //Button to align Limelight

    // Preset aim constants
    public static final double LOWRPM = 0.0; // RPM that the LOW aimMode winds to
    public static final double LOWAngle = 0.0; // Angle that the LOW aimMode adjusts to
    public static final double LAUNCHRPM = 0.0; // RPM that the LAUNCH aimMode winds to
    public static final double LAUNCHAngle = 0.0; // Angle that the LAUNCH aimMode adjusts to
    public static final double TARMACRPM = 0.0; // RPM that the TARMAC aimMode winds to
    public static final double TARMACAngle = 0.0; // Angle that the TARMAC aimMode adjusts to
    
    // Shooter Constants
    public static final int shooterID = 7; // ID of the shooter
    public static final int hoodID = 8; // ID of the hood;
    public static final double shooterHeight = 3;
    public static final double highHeight = 8.0 + 8.0/12.0; // Height of the high goal in ft from the carpet
    public static final double lowHeight = 5.0 + 7.75/12.0; // Height of the low goal in ft from the carpet
    public static final double airboneTime = 1.2;
    public static final double gravity = 32;
    public static final double LLHeight = 8.5/12.0; // Height of the limelight in ft from the carpet
    public static final double LLAngle = 29.0; // Angle that the limelight is mounted at from a vertical plane, ensure this is as exact as possible
    public static final double gearRatioIn = 34;
    public static final double gearRatioOut = 14;
    public static final double gearDiameter = 4; // Gear diameter in inches
    public static final double ballFlywheelratio = 2;
    public static final int shooterCargoID = 9;
    public static final double cargoRotation = 3;
}