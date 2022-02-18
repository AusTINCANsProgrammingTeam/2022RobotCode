// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AimModes;

import java.lang.Math;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.common.hardware.MotorController;

public class ShooterSubsystem extends SubsystemBase {
  private MotorController flywheelController;
  private MotorController hoodController;
  private SparkMaxPIDController flywheelPID;
  private SparkMaxPIDController hoodPID;
  private RelativeEncoder flywheelEncoder;
  private RelativeEncoder hoodEncoder;
  private MotorController stopperController;
  private AimModes aimMode;
  private double currentRPM;
  private double RPMIN = 3500;

  public ShooterSubsystem() {
    SmartDashboard.putNumber("RPMIN", RPMIN);
    aimMode = AimModes.TEST;
    // Initializes the SparkMAX for the flywheel
    flywheelController = new MotorController("Flywheel", Constants.shooterID, 40, true);
    flywheelPID = flywheelController.getPID();
    flywheelEncoder = flywheelController.getEncoder();
    // Initializes the SparkMAX for the hood
    hoodController = new MotorController("Hood", Constants.hoodID);
    hoodPID = hoodController.getPID();
    hoodEncoder = hoodController.getEncoder();
    // Initializes the SparkMAX for the cargo stopper
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
    aimMode = AimModes.values()[m];
  }

  public void cycleAimModeNext() {
    aimMode.next();
  }

  public void cycleAimModePrevious() {
    aimMode.previous();
  }

  public double getTY() {
    // Gets TY, the vertical angle of the target from the limelight
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getDistance() {
    // Uses Limelight to find distance to High Goal
    SmartDashboard.putNumber("ty", getTY());
    return (Constants.highHeight - Constants.LLHeight) / Math.tan(Math.toRadians((getTY() + Constants.LLAngle))); // Return distance in ft
  }

  public double UnitConversion(double KBallSpeed, double GearDiameter) {
    // Convert from FPS of the ball into RPM
    return (((KBallSpeed * 12) / Constants.gearDiameter) * Constants.ballFlywheelratio) * 2;
  }

  public void prime() {
    // Check what aimMode is active, gets distance if AUTO, winds flywheel, adjusts
    // hood correspondingly
    switch (aimMode) {
      case EJECT: // aimMode used to eject unwanted balls from the shooter
        adjustHood(aimMode.getAngle());
        windFlywheel(aimMode.getRPM());
        break;
      case LOW: // aimMode used to dump into the low goal from ~1ft
        adjustHood(aimMode.getAngle());
        windFlywheel(aimMode.getRPM());
        break;
      case TARMAC: // aimMode used to shoot into the high goal from ~2ft
        adjustHood(aimMode.getAngle());
        windFlywheel(aimMode.getRPM());
        break;
      case LAUNCH: // aimMode used to shoot into the high goal from the launchpad
        adjustHood(aimMode.getAngle());
        windFlywheel(aimMode.getRPM());
        break;
      case AUTO: // aimMode used to automatically shoot into the high goal

        break;
      case TEST: // aimMode to take a RPM from the dashboard
        windFlywheel(SmartDashboard.getNumber("RPMIN", RPMIN));
        break;
    }
  }

  public void updateSmartDashboard() {
    SmartDashboard.putString("AimMode", "");
    SmartDashboard.putNumber("IAccum",flywheelPID.getIAccum());
    SmartDashboard.putNumber("Distance", getDistance());
    SmartDashboard.putNumber("RPM", flywheelEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
