// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.opencv.core.Mat;
import com.revrobotics.RelativeEncoder;
import frc.robot.common.hardware.MotorController;

public class ShooterSubsystem extends SubsystemBase {
  private int aimMode; // 0 is LOW, 1 is AUTO, 2 is LAUNCH, 3 is TARMAC, 4 is TEST
  private MotorController shooter_motorController;
  private MotorController hood_motorController;
  private SparkMaxPIDController KShooterController;
  private SparkMaxPIDController KHoodController;
  private RelativeEncoder KShooterEncoder;
  private RelativeEncoder KHoodEncoder;

  public ShooterSubsystem() {
    aimMode = 1;
    //Motor initialization
    shooter_motorController = new MotorController("Shooter", Constants.kShooterID);
    KShooterController = shooter_motorController.getPID();
    KShooterEncoder = shooter_motorController.getEncoder();
    hood_motorController = new MotorController("Hood", Constants.kHoodID);
    KHoodController = hood_motorController.getPID();
    KHoodEncoder = hood_motorController.getEncoder();
  }

  public void adjustHood(double a) {
    double kNumOfRotation = a * 2;

    // Adjusts hood using PID control to passed angle a
    KHoodController.setReference(kNumOfRotation, CANSparkMax.ControlType.kPosition);
  }

  public void windFlywheel(int rpm) {
    KShooterController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    // Winds flywheel using PID control to passed rpm
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
    return (Constants.kHighHeight - Constants.kLLHeight) / Math.tan(getTY() + Constants.kLLAngle); // Return distance in
                                                                                                   // feet
  }

  public double[] ProjectilePrediction(double y0, double x0, double y, double x, double g, double t) {
    x = x + 1;
    y = y + 0.4;

    double Fangle = Math.toDegrees(Math.atan((y - y0 + 1 / 2 * g * (Math.pow(t, 2))) / x));
    double Velocity1 = Math.abs(x / (Math.cos(Math.toRadians(Fangle)) * t));
    double[] VandA = new double[2];
    VandA[0] = FlywheelBallConversion(Velocity1);
    VandA[1] = Fangle;
    return VandA;

  }

  public double FlywheelBallConversion(double KBallSpeed) {
    return KBallSpeed * 2;
    // Convert from Ft/Second of the ball into RPM
    // Convert From Ball Speed to required FlyWheel Speed
    // Convert from required Flywheel speed to motor speed (Gear Ratio)

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
      case 1: // Case for AUTO mode, calculates trajectory and winds flywheel/adjusts hood to a dynamic state
        adjustHood(ProjectilePrediction(Constants.kShooterHeight, 0, Constants.kHighHeight, getDistance(), 32,
            Constants.kAirboneTime)[1]);
        windFlywheel((int) (Math.ceil(ProjectilePrediction(Constants.kShooterHeight, 0, Constants.kHighHeight,
            getDistance(), Constants.kGravity, Constants.kAirboneTime)[0])));
        // Maybe break it down in the future for visibility
        break;
      case 2: // Case for LAUNCH mode, winds flywheel to preset RPM and adjusts hood to preset angle
        adjustHood(Constants.kLAUNCHAngle);
        windFlywheel(Constants.kLAUNCHRPM);
        break;
      case 3: // Case for TARMAC mode, winds flywheel to preset RPM and adjusts hood to preset angle
        adjustHood(Constants.kTARMACAngle);
        windFlywheel(Constants.kTARMACRPM);
        break;
      case 4: //Case for TEST mode, just takes an RPM from shuffleboard and winds
        windFlywheel(200);
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}