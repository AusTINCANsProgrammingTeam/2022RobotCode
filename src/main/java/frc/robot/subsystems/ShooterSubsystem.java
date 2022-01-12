// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private int AimMode; //0 is LOW, 1 is AUTO, 2 is LAUNCH, 3 is TARMAC

  public ShooterSubsystem() {
    AimMode = 1;
  }

  public void adjustHood(double a) {
    //Adjusts Hood using PID control to passed angle a
  }

  public void windFlywheel(int rpm) {
    //Winds Flywheel using PID control to passed rpm
  }

  public void setAimMode(int m) {
    AimMode = m;
  }

  public void cycleAimModeUp() {
    AimMode++;
    if(AimMode > 3){AimMode = 0;}
  }

  public void cycleAimModeDown() {
    AimMode--;
    if(AimMode < 0){AimMode = 3;}
  }

  public double getDistance() {
    //Uses Limelight to find distance to High Goal
    return 0.0; //Return distance in feet
  }

  public void prime() {
    //Check what aimMode is active, gets distance if AUTO, winds flywheel, adjusts hood correspondingly
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
