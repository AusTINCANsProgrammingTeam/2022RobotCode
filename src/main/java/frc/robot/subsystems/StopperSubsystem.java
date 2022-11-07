// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StopperConstants;
import frc.robot.common.hardware.MotorController;
import frc.robot.common.hardware.MotorController.MotorConfig;

public class StopperSubsystem extends SubsystemBase {
  private CANSparkMax stopperWheelMotor;

  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry stopperRunning =
      operatorTab.add("Stopper Running", false).withPosition(2, 0).getEntry();

  public StopperSubsystem() {
    stopperWheelMotor = MotorController.constructMotor(MotorConfig.stopperWheel);
  }

  public void forward() {
    stopperWheelMotor.set(StopperConstants.forwardSpeed);
    stopperRunning.setBoolean(true);
  }

  public void reverse() {
    stopperWheelMotor.set(StopperConstants.reverseSpeed);
    stopperRunning.setBoolean(true);
  }

  public void stop() {
    stopperWheelMotor.set(0);
    stopperRunning.setBoolean(false);
  }

  @Override
  public void periodic() {
  }
}
