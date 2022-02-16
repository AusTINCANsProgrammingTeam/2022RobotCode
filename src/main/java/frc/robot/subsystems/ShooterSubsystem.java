// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.Math;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.common.hardware.MotorController;

public class ShooterSubsystem extends SubsystemBase {
  private int aimMode; // 0 is LOW, 1 is AUTO, 2 is LAUNCH, 3 is TARMAC, 4 is TEST
  private MotorController flywheelController;
  private MotorController hoodController;
  private SparkMaxPIDController flywheelPID;
  private SparkMaxPIDController hoodPID;
  private RelativeEncoder flywheelEncoder;
  private RelativeEncoder hoodEncoder;
  private MotorController stopperController;
  private double currentRPM;
  private double RPMIN = 3500;

  public ShooterSubsystem() {
    SmartDashboard.putNumber("RPMIN", RPMIN);
    aimMode = 4;
    // Initializes the SparkMAX for the flywheel
    flywheelController = new MotorController("Flywheel", Constants.shooterID, 40, true);
    flywheelPID = flywheelController.getPID();
    flywheelEncoder = flywheelController.getEncoder();
    // Initializes the SparkMAX for the hood
    hoodController = new MotorController("Hood", Constants.hoodID);
    hoodPID = hoodController.getPID();
    hoodEncoder = hoodController.getEncoder();
    // Initializes the SparkMax for the cargo stopper
    stopperController = new MotorController("Shooter Cargo", Constants.shooterCargoID);
    // Initializes PID for the flywheel
    flywheelPID.setP(5e-4);
    flywheelPID.setI(6e-7);
    flywheelPID.setIMaxAccum(0.9, 0);
    flywheelPID.setD(0.0);
    flywheelPID.setOutputRange(0, 1);
    // Initializes PID for the hood
    hoodPID.setP(0.0);
    hoodPID.setI(0.0);
    hoodPID.setD(0.0);
    hoodPID.setOutputRange(0, 1);
  }

  public void adjustHood(double a) {
    // Adjusts Hood using PID control to passed angle a
    hoodPID.setReference(a, CANSparkMax.ControlType.kPosition);
  }

  public void windFlywheel(double rpm) {
    // Winds Flywheel using PID control to passed rpm
    if(rpm == 0){
      flywheelPID.setReference(0, CANSparkMax.ControlType.kVoltage);
      flywheelPID.setIAccum(0);
    } else{
    currentRPM = rpm;
    flywheelPID.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }
  }

  public void runCargo(boolean a,boolean reversed) {
    if(a){
      if(reversed){stopperController.setSpeed(-0.2);}
      else{stopperController.setSpeed(0.2);}
    }else{
      stopperController.setSpeed(0.0);
    }

  }

  public boolean wheelReady(){
    double flywheelSpeed = flywheelEncoder.getVelocity();
    if (flywheelSpeed > currentRPM - 10 && flywheelSpeed < currentRPM + 10) {
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
      case 4: // Case for TEST mode, just takes an RPM and winds
        windFlywheel(SmartDashboard.getNumber("RPMIN", RPMIN));
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IAccum",flywheelPID.getIAccum());
    SmartDashboard.putNumber("dist", getDistance());
    SmartDashboard.putNumber("RPM", flywheelEncoder.getVelocity());
  }

}
