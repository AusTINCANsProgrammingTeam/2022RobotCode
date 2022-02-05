// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.lang.Math;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.common.hardware.MotorController;

public class ShooterSubsystem extends SubsystemBase {
  private int aimMode; // 0 is LOW, 1 is AUTO, 2 is LAUNCH, 3 is TARMAC, 4 is TEST
  private MotorController shooter_motorController;
  private MotorController hood_motorController;
  private SparkMaxPIDController KShooterController;
  private SparkMaxPIDController KHoodController;
  private RelativeEncoder KShooterEncoder;
  private RelativeEncoder KHoodEncoder;
  private MotorController cargo_motorController;
  private SparkMaxPIDController kCargoController;
  private RelativeEncoder kCargoEncoder;
  private double currentRPM;

  public ShooterSubsystem() {

    aimMode = 4;
    cargo_motorController = new MotorController("Shooter Cargo", Constants.shooterCargoID);
    kCargoEncoder = cargo_motorController.getEncoder();

    shooter_motorController = new MotorController("Shooter", Constants.shooterID, 40, true);
    KShooterController = shooter_motorController.getPID();
    KShooterEncoder = shooter_motorController.getEncoder();
    KShooterController.setP(6e-4);
    KShooterController.setI(1e-6);
    KShooterController.setD(0.0);
/*
    hood_motorController = new MotorController("Hood", Constants.hoodID);
    KHoodController = hood_motorController.getPID();
    KHoodEncoder = shooter_motorController.getEncoder();*/

  }

  public void adjustHood(double a) {
    KHoodController.setReference(a, CANSparkMax.ControlType.kPosition);

    // Adjusts Hood using PID control to passed angle a
  }

  public void windFlywheel(double rpm) {
    // Winds Flywheel using PID control to passed rpm
    // double adjustedRPM = rpm * (Constants.kGearRatioIn / Constants.kGearRatioOut); TODO: reconsider using this
    currentRPM = rpm;
    KShooterController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
  }

  public void runCargo(boolean a) {
    if(a){
      cargo_motorController.setSpeed(1.0);
    }else{
      cargo_motorController.setSpeed(0.0);
    }
  }

  public boolean wheelReady(){
    double flywheelSpeed = KShooterEncoder.getVelocity();
    if (flywheelSpeed > currentRPM - 15 && flywheelSpeed < currentRPM + 15) {
      return true;
    }
    return false;
  }

  public void setAimMode(int m) {
    aimMode = m;
  }

  public void cycleAimModeUp() {
    aimMode++;
    if (aimMode > 3) {
      aimMode = 0;
    }
  }

  public void cycleAimModeDown() {
    aimMode--;
    if (aimMode < 0) {
      aimMode = 3;
    }
  }

  public double getTY() {
    // Gets TY, the vertical angle of the target from the limelight
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getDistance() {
    // Uses Limelight to find distance to High Goal
    SmartDashboard.putNumber("ty", getTY());
    return (Constants.highHeight - Constants.LLHeight) / Math.tan(Math.toRadians((getTY() + Constants.LLAngle))); // Return distance in
                                                                                                   // feet
  }

  public double[] ProjectilePrediction(double y0, double x0, double y, double x, double g, double t) {
    x = x + 1; // Applies an offset to target goal center
    double hoodAngle = Math.toDegrees(Math.atan((y - y0 + 1 / 2 * g * (Math.pow(t, 2))) / x)); //Finds angle to shoot at
    double velocity = Math.abs(x / (Math.cos(Math.toRadians(hoodAngle)) * t)); //Finds velocity in feet per second to shoot at
    double[] returnArray = new double[2];
    returnArray[0] = UnitConversion(velocity, Constants.gearDiameter); //Converts FPS to RPM
    returnArray[1] = hoodAngle;
    return returnArray;

  }

  public double UnitConversion(double KBallSpeed, double GearDiameter) {
    // Convert from FPS of the ball into RPM
    return (((KBallSpeed * 12) / Constants.gearDiameter) * Constants.ballFlywheelratio) * 2;
  }

  public void prime() {
    // Check what aimMode is active, gets distance if AUTO, winds flywheel, adjusts
    // hood correspondingly
    switch (aimMode) {
      case 0: // Case for LOW mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.LOWAngle);
        windFlywheel(Constants.LOWRPM);
        break;
      case 1: // Case for AUTO mode, calculates trajectory and winds flywheel/adjusts hood to
              // a dynamic state
        adjustHood(ProjectilePrediction(Constants.shooterHeight, 0, Constants.highHeight, getDistance(),
            Constants.gravity, Constants.airboneTime)[1]);

        windFlywheel((int) (Math.ceil(ProjectilePrediction(Constants.shooterHeight, 0, Constants.highHeight,
            getDistance(), 32, Constants.airboneTime)[0])));

        break;
      case 2: // Case for LAUNCH mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.LAUNCHAngle);
        windFlywheel(Constants.LAUNCHRPM);
        break;
      case 3: // Case for TARMAC mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.TARMACAngle);
        windFlywheel(Constants.TARMACRPM);
        break;
      case 4: //Case for TEST mode, just takes an RPM and winds
        windFlywheel(3000.0);
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("dist", getDistance());
  }

}
