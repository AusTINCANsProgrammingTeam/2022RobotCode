// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class ClimbSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private MotorController m_climbMotorControllerOne;
  private MotorController m_climbMotorControllerTwo;
  private DigitalInput m_limitSwitch;
  

  // TODO: maybe add servos

  public ClimbSubsystem() {
    //One is left, two is right
    m_climbMotorControllerOne = new MotorController("Climb Motor One", Constants.kClimbMotorOne, 40, true);
    m_climbMotorControllerTwo = new MotorController("Climb Motor Two", Constants.kClimbMotorTwo, 40, true);
    m_climbMotorControllerTwo.setInverted(true);
    m_climbMotorControllerTwo.setFollow(m_climbMotorControllerOne);

    m_limitSwitch = new DigitalInput(Constants.kLimitSwitchChannel);

  }

  public void enableClimb(boolean on, boolean up){
    if (on) {
      if (up) {
        m_climbMotorControllerOne.getEncoder().setPosition(0);
        m_climbMotorControllerOne.getPID().setReference(25, CANSparkMax.ControlType.kPosition);
      } else {
        m_climbMotorControllerOne.getEncoder().setPosition(0);
        m_climbMotorControllerOne.getPID().setReference(-25, CANSparkMax.ControlType.kPosition);
      }
    } else {
      m_climbMotorControllerOne.setSpeed(0);
    }
  }

  public void periodic() {
    SmartDashboard.putNumber("Climb Hight", m_climbMotorControllerOne.getEncoder().getPosition());
  }

  public boolean getLimitSwitchVal() {
      return m_limitSwitch.get();
  }
  // TODO: might add other getter methods depending on how many limit switches 
}