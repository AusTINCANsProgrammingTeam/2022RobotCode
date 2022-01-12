// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;

public class ShooterSubsystem extends SubsystemBase {
  private int aimMode; //0 is LOW, 1 is AUTO, 2 is LAUNCH, 3 is TARMAC
  private double ty; //Angle of target from Limelight, accessed via NetworkTables

  public ShooterSubsystem() {
    aimMode = 1;
  }

  public void adjustHood(double a) {
    //Adjusts Hood using PID control to passed angle a
  }

  public void windFlywheel(int rpm) {
    //Winds Flywheel using PID control to passed rpm
  }

  public void setAimMode(int m) {
    aimMode = m;
  }

  public void cycleAimModeUp() {
    aimMode++;
    if(aimMode > 3){aimMode = 0;}
  }

  public void cycleAimModeDown() {
    aimMode--;
    if(aimMode < 0){aimMode = 3;}
  }

  public double getDistance() {
    //Uses Limelight to find distance to High Goal
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    return (8.8 - Constants.kLLHeight) / Math.tan(ty + Constants.kLLAngle); //Return distance in feet
  }

  public void prime() {
    //Check what aimMode is active, gets distance if AUTO, winds flywheel, adjusts hood correspondingly
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
