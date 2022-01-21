// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.lang.Math;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import org.opencv.core.Mat;
import com.revrobotics.RelativeEncoder;
import frc.robot.common.hardware.MotorController;

public class ShooterSubsystem extends SubsystemBase {
  private int aimMode; // 0 is LOW, 1 is AUTO, 2 is LAUNCH, 3 is TARMAC
  private MotorController shooter_motorController;
  private MotorController hood_motorController;
  private SparkMaxPIDController KShooterController;
  private SparkMaxPIDController KHoodController;
  private RelativeEncoder KShooterEncoder;
  private RelativeEncoder KHoodEncoder;
  private MotorController cargo_motorController;
  private SparkMaxPIDController kCargoController;
  private RelativeEncoder kCargoEncoder;
  private int shooterRPM;
  private NetworkTableEntry sbShooterRPM;
  private int currentRPM;

  public ShooterSubsystem() {
    sbShooterRPM = RobotContainer.debugTab.add("shooterRPM", 0).getEntry();

    aimMode = 1;
    cargo_motorController = new MotorController("Shooter Cargo", Constants.kShooterCargoID);
    kCargoController = cargo_motorController.getPID();
    kCargoEncoder = cargo_motorController.getEncoder();

    shooter_motorController = new MotorController("Shooter", Constants.kShooterID);
    KShooterController = shooter_motorController.getPID();
    KShooterEncoder = shooter_motorController.getEncoder();
    hood_motorController = new MotorController("Hood", Constants.kHoodID);
    KHoodController = hood_motorController.getPID();
    KHoodEncoder = shooter_motorController.getEncoder();

  }

  public void adjustHood(double a) {
    KHoodController.setReference(a, CANSparkMax.ControlType.kPosition);

    // Adjusts Hood using PID control to passed angle a
  }

  public void windFlywheel(int rpm) {
    // Winds Flywheel using PID control to passed rpm
    double kGearRationRPM = rpm * (Constants.kGearRatioIn / Constants.kGearRatioOut);
    currentRPM = rpm;
    KShooterController.setReference(kGearRationRPM, CANSparkMax.ControlType.kVelocity);
  }

  public void shoot() { //uncomment when merged; missing MotorController method
    /*cargo_motorController.setSpeed(1.0);
    Thread.sleep(200);
    cargo_motorController.setSpeed(0.0);*/
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
    return Math.toDegrees(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
  }

  public double getDistance() {
    // Uses Limelight to find distance to High Goal
    return (Constants.kGoalHeight - Constants.kLLHeight) / Math.tan(getTY() + Constants.kLLAngle); // Return distance in
                                                                                                   // feet
  }

  public double[] ProjectilePrediction(double y0, double x0, double y, double x, double g, double t) {
    /**
     * Fangle =math.degrees(math.atan((y-y0+1/2*g*(t**2))/x))
     * velcocity1 = abs(x/(math.cos(math.radians(Fangle))*t))
     * velcocity2 = abs((y-y0+(g/2.0)*(t**2))/(math.sin(math.radians(Fangle))*t))
     * return Fangle, velcocity1, velcocity2
     */
    x = x + 1; // Applies an offset to target goal center
    double Fangle = Math.toDegrees(Math.atan((y - y0 + 1 / 2 * g * (Math.pow(t, 2))) / x));
    double Velocity1 = Math.abs(x / (Math.cos(Math.toRadians(Fangle)) * t));
    double Velocity2 = Math.abs((y - y0 + (g / 2.0) * (Math.pow(t, 2))) / (Math.sin(Math.toRadians(Fangle)) * t));
    double[] VandA = new double[2];
    VandA[0] = UnitConversion(Velocity1, Constants.kGearDiameter);
    VandA[1] = Fangle;
    return VandA;

  }

  public double UnitConversion(double KBallSpeed, double GearDiametter) {
    double KFlywheelrpm = ((KBallSpeed * 12) / Constants.kGearDiameter) * Constants.kBallFlywheelratio;
    return KFlywheelrpm;
    // Convert from Ft/Second of the ball into RPM

  }

  public void prime() {
    // Check what aimMode is active, gets distance if AUTO, winds flywheel, adjusts
    // hood correspondingly
    switch (aimMode) {
      case 0: // Case for LOW mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.kLOWAngle);
        windFlywheel(Constants.kLOWRPM);
        break;
      case 1: // Case for AUTO mode, calculates trajectory and winds flywheel/adjusts hood to
              // a dynamic state
        adjustHood(ProjectilePrediction(Constants.kShooterHeight, 0, Constants.kHighHeight, getDistance(),
            Constants.kGravity, Constants.kAirboneTime)[1]);

        windFlywheel((int) (Math.ceil(ProjectilePrediction(Constants.kShooterHeight, 0, Constants.kHighHeight,
            getDistance(), 32, Constants.kAirboneTime)[0])));

        break;
      case 2: // Case for LAUNCH mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.kLAUNCHAngle);
        windFlywheel(Constants.kLAUNCHRPM);
        break;
      case 3: // Case for TARMAC mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.kTARMACAngle);
        windFlywheel(Constants.kTARMACRPM);
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterRPM = sbShooterRPM.getNumber(Integer.valueOf(shooterRPM)).intValue();
    //System.out.println("shooterRPM is "+shooterRPM);

  }
}