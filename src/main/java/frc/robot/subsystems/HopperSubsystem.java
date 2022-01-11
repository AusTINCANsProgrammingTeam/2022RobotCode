// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

/** Add your docs here. */
public class HopperSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax m_motorController3 = new CANSparkMax(Constants.kHopperMotorThreeID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax m_shoooterMotorController = new CANSparkMax(Constants.kShooterMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  public void HopperSwitch(boolean on) {
    if (on) {
      m_motorController3.set(Constants.kHopperMotorSpeed); 
    } else {
      m_motorController3.set(0);
    }
  }

  /*
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  */
}
