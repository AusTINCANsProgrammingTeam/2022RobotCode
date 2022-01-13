// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
// constants. This class should not be used for any other purpose. All constants should be declared
// globally (i.e. public static). Do not put anything functional in this class.

// <p>It is advised to statically import this class (or one of its inner classes) wherever the
// constants are needed, to reduce verbosity.

public final class Constants {
        
    // Constants for wheel motors

    // Actual IDs on robot, used to activate the right motors
    // TODO: Values to be changed, these are placeholder values for now
    public static final int kDriveRightFront = 0;
    public static final int kDriveRightMiddle = 0;
    public static final int kDriveRightRear = 0;
    public static final int kDriveLeftFront = 0;
    public static final int kDriveLeftMiddle = 0;
    public static final int kDriveLeftRear = 0;
    
    // This is used for organizational purposes (Note numbers 0-5 to distinguish between the 6 motors)
    public static final int kDriveLeftFrontIndex = 0;
    public static final int kDriveLeftMiddleIndex = 1;
    public static final int kDriveLeftRearIndex = 2;
    public static final int kDriveRightFrontIndex = 3;
    public static final int kDriveRightMiddleIndex = 4;
    public static final int kDriveRightRearIndex = 5;

    public static final int kDriveBaseCurrentLimit = 40;

    //Controller constants
    public static final int kDBJoystickPort = 0;

    public static final int kDBLeftJoystickAxisX = 0;
    public static final int kDBLeftJoystickAxisY = 1;
    public static final int kDBRightJoystickAxisX = 2;
    public static final int kDBRightJoystickAxisY = 3;

    // Intake Contstants 
    public static final int kIntakeMotorOneID = 11;
    public static final int kIntakeMotorTwoID = 12;
    public static final double kIntakeMotorSpeed = 0.25;

    //Hopper Constants
    public static final int kHopperMotorThreeID = 13;
    public static final double kHopperMotorSpeed = 0.25;
    
    //Joystick Constants
    public static final int kPortNumber = 0;
    public static final int kJoystickButtonNumberOne = 1;
    public static final int kJoystickButtonNumberTwo = 2;
    public static final int kAButton = 1;           // Button for hopper
    public static final int kRightBumperButton = 5; // Button for intake 
    public static final int kLeftBumperButton = 6;  // Button to reverse intake
}
