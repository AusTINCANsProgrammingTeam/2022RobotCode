// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double LLHeight = 1.5; //Height of the limelight in ft from the carpet
    public static final double LLAngle = 30.0; //Angle that the limelight is mounted at
    public static final double goalHeight = 8.8; //Height of the goal in ft from the carpet
  
    // TODO: Swap placeholder motor IDs & Button Ports and Numbers for real ones 
    // Constants for wheel motors
  
    // Actual IDs on robot, used to activate the right motors
    // TODO: Values to be changed, these are placeholder values for now
    public static final int driveRightFront = 21;
    public static final int driveRightMiddle = 22;
    public static final int driveRightRear = 23;
    public static final int driveLeftFront = 24;
    public static final int driveLeftMiddle = 25;
    public static final int driveLeftRear = 26;
    
    // This is used for organizational purposes (Note numbers 0-5 to distinguish between the 6 motors)
    public static final int driveLeftFrontIndex = 0;
    public static final int driveLeftMiddleIndex = 1;
    public static final int driveLeftRearIndex = 2;
    public static final int driveRightFrontIndex = 3;
    public static final int driveRightMiddleIndex = 4;
    public static final int driveRightRearIndex = 5;

    public static final int driveBaseCurrentLimit = 40;

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
    public static final int CDSAlignmentID = 15;
    public static final double CDSBeltSpeed = 0.15;
    public static final double CDSWheelControllerSpeed = 0.25;
    public static final double CDSAlignmentSpeed = 0.25;

    //Controller constants
    public static final int DBLeftJoystickAxisX = 0;
    public static final int DBLeftJoystickAxisY = 1;
    public static final int DBRightJoystickAxisX = 2;
    public static final int DBRightJoystickAxisY = 3;

    //Joystick Constants
    public static final int portNumber = 0; //driver joystick
    public static final int BButton = 2; // Button for forward CDS
    public static final int XButton = 3; // Button for reverse CDS
    public static final int AButton = 1; // Button for Alignment CDS
    public static final int rightBumperButton = 5; // Button for intake 
    public static final int leftBumperButton = 6;  // Button to reverse intake
}
