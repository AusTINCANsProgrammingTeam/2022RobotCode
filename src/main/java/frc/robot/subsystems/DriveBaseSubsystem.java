// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.hardware.MotorController;


public class DriveBaseSubsystem extends SubsystemBase {

  private final Joystick m_driverJoystick;
  private final MotorController[] m_motorControllers = new MotorController[6];
  private final DifferentialDrive m_differentialDrive;
  
  public DriveBaseSubsystem(Joystick joystick) {  
    m_driverJoystick = joystick;
    
    m_differentialDrive = new DifferentialDrive(m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax(), m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
    m_differentialDrive.setRightSideInverted(true); // need to invert the right side (no longer does it by default in 2022)
    m_motorControllers[Constants.kDriveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.kDriveLeftFront);
    
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
