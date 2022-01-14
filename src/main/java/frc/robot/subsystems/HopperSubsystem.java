// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.hardware.MotorController;

/** Add your docs here. */
public class HopperSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController[] mHopperController = new MotorController[1];

  public HopperSubsystem() {
    mHopperController[Constants.kHopperMotorThreeIndex] = new MotorController("Hopper Motor", Constants.kHopperMotorThreeID);
  }

  public void HopperSwitch(boolean on) {
    if (on) {
      mHopperController[Constants.kHopperMotorThreeIndex].getSparkMax().set(Constants.kHopperMotorSpeed);
      SmartDashboard.putNumber("Hopper Motor Speed", Constants.kHopperMotorSpeed);
    } else {
      mHopperController[Constants.kHopperMotorThreeIndex].getSparkMax().set(0.0);
      SmartDashboard.putNumber("Hopper Motor Speed", 0);
    }
  }

  public void ForwardHopper() {
    mHopperController[Constants.kHopperMotorThreeIndex].getSparkMax().setInverted(false);
    SmartDashboard.putString("Hopper Motor Direction", "Forward");
  }

  public void ReverseHopper() {
    mHopperController[Constants.kHopperMotorThreeIndex].getSparkMax().setInverted(true);
    SmartDashboard.putString("Hopper Motor Direction", "Reverse");
  }

}