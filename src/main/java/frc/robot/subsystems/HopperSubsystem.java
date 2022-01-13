// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class HopperSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax mHopperController;

  public HopperSubsystem() {
    mHopperController = new CANSparkMax(Constants.kHopperMotorThreeID, CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  public void HopperSwitch(boolean on) {
    if (on) {
      mHopperController.set(Constants.kHopperMotorSpeed);
      SmartDashboard.putNumber("Hopper Motor Speed", Constants.kHopperMotorSpeed);
    } else {
      mHopperController.set(0.0);
      SmartDashboard.putNumber("Hopper Motor Speed", 0);
    }
  }

}