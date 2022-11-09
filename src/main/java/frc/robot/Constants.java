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
  public static final boolean competitionRobot = true; //Set based on whether the competition robot or practice robot is in use

  public static final class MotorDefaults{
    //Constants to use as default values for Motor Controllers
    public static final int kCurrentLimit = 40;
    public static final double kOpenLoopRampRate = 0.2;
  }

  // TODO: when true, shooter is over shooting
  public static final boolean DebugMode = false; // TODO: change to false for competition time

  // Constants for wheel motors
  public static final double wheelRadius = 2; // radius of wheel, use for calculating angular values
  // Rate at which the motors reach maximum speed; TODO: tune for optimal performance
  public static final double gearRatio = 6.2; // 10.75 : 1 gear ratio <--- kitbot
  // 10.75 motor rotations : 1 wheel rotation
  public static final double inchesInMeter = 39.3701;

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
  // TODO: Change to true when using external encoders
  public static final boolean usingExternal = false;
  public static final int encoderCountsPerRev = 8192;

  public static final class IntakeConstants{
    public static final double[] PIDArray = {0.25, 0, 1};
    public static final double deployPosition = 72;
    public static final double retractPosition = 5;
    public static final double intakeSpeed = 1.0;
  }

  public static final class CDSConstants{
    public static final double singulatorSpeed = 0.80;
    public static final double beltSpeed = 0.75;
  }

  public static final class StopperConstants {
    public static final double forwardSpeed = 1.0;
    public static final double reverseSpeed = -0.4;
  }
 
  public static final class ShooterConstants {
    public static final double fenderRPM = 1400;

    // PID settings
    public static final double kPIDArray[] = {1.4e-09, 3.0E-8, 0};
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

  public static final class ClimbConstants {
    public static final int armMaxPosition = 132;
    public static final int armMinPosition = 0;
    public static final int poleMaxPosition = 6;
    public static final int poleMinPosition = -60;
    public static final int poleDeployPosition = -12;

    public static final double[] armPIDArray = {0.125, 0, 0};
    public static final double armIMaxAccum = 0.45;
    public static final double[] polePIDArray = {0.125, 0, 0};
    public static final double poleIMaxAccum = 0.25;

    public static final double PIDMaxOutput = 1.0;

    public static final int armLowCurrent = 10;
    public static final int armHighCurrent = 60;
    public static final int poleLowCurrent = 15;
    public static final int poleHighCurrent = 50;

    public static final int servoOneID = 0;
    public static final int servoTwoID = 1;
    public static final double servoOneLocked = 0.35;
    public static final double servoOneUnlocked = 1;
    public static final double servoTwoLocked = 0.65;
    public static final double servoTwoUnlocked = 0;

    public static final double armUpSpeed = -1;
    public static final double armDownSpeed = -0.65;
    public static final double poleInSpeed = 0.5;
    public static final double poleOutSpeed = 0.6;
  }
}
