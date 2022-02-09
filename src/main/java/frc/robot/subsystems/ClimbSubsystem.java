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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;


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
    m_climbMotorControllerOne = new MotorController("Climb Motor One", Constants.kClimbMotorOneIndex);

    m_climbMotorControllerTwo = new MotorController("Climb Motor Two", Constants.kClimbMotorTwoIndex);
    m_climbMotorControllerTwo.setInverted(true);
    m_climbMotorControllerTwo.setFollow(m_climbMotorControllerOne);

    m_limitSwitch = new DigitalInput(Constants.kLimitSwitchChannel);

    //Example camera system, unsure if it goes here or not
    CameraServer.startAutomaticCapture();
    //Not used at the moment, maby in the futue
    CvSink cvSink = CameraServer.getVideo();
    CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
  }

  public void enableClimb(boolean on, boolean up){
    if (on) {
      if (up) {
        m_climbMotorControllerOne.getPID().setReference(25, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Climb Hight", m_climbMotorControllerOne.getEncoder().getPosition());
      } else {
        m_climbMotorControllerOne.getPID().setReference(-25, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Climb Hight", m_climbMotorControllerOne.getEncoder().getPosition());
      }
    } else {
      m_climbMotorControllerOne.setSpeed(0);
    }
  }

  public boolean getLimitSwitchVal() {
      return m_limitSwitch.get();
  }
  // TODO: might add other getter methods depending on how many limit switches 
}