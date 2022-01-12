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

// TODO: Swap placeholder motor IDs & Button Ports and Numbers for real ones 
public final class Constants {
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
    public static final int kAButton = 1; // Button for hopper
    public static final int kRightBumperButton = 5; // Button for intake 
    public static final int kLeftBumperButton = 6; // Button to reverse intake
    
    //Distance Calculation Constants
    public static final double kLLHeight = 1.5; //Height of the limelight in ft from the carpet
    public static final double kLLAngle = 30.0; //Angle that the limelight is mounted at

}
