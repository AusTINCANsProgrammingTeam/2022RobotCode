// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class DriveBaseSubsystem extends SubsystemBase {

  private final Joystick m_driverJoystick;
  private final MotorController[] m_motorControllers = new MotorController[6];
  private final DifferentialDrive m_differentialDrive;

  
  public DriveBaseSubsystem(Joystick joystick) {  
    m_driverJoystick = joystick;
    
    m_differentialDrive = new DifferentialDrive(m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax(), m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
    
    // motor controllers
    m_motorControllers[Constants.kDriveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.kDriveLeftFront);
    m_motorControllers[Constants.kDriveLeftMiddleIndex] = new MotorController("Differential Left Middle", Constants.kDriveLeftMiddle);
    m_motorControllers[Constants.kDriveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.kDriveLeftRear);
    m_motorControllers[Constants.kDriveRightFrontIndex] = new MotorController("Differential Right Front", Constants.kDriveRightFront);
    m_motorControllers[Constants.kDriveRightMiddleIndex] = new MotorController("Differential Right Middle", Constants.kDriveRightMiddle);
    m_motorControllers[Constants.kDriveRightRearIndex] = new MotorController("Differential Right Rear", Constants.kDriveRightRear);

    // inverses right side motors (2022 wpilib doesn't default it to be inverted for differential drive)
    m_motorControllers[Constants.kDriveRightFrontIndex].setInverted(true);
    m_motorControllers[Constants.kDriveRightMiddleIndex].setInverted(true);
    m_motorControllers[Constants.kDriveRightRearIndex].setInverted(true);

    //Forces middle and rear motors of each side to follow the first
    m_motorControllers[Constants.kDriveLeftRearIndex].setFollow(m_motorControllers[Constants.kDriveLeftFrontIndex]);
    m_motorControllers[Constants.kDriveLeftMiddleIndex].setFollow(m_motorControllers[Constants.kDriveLeftFrontIndex]);
    m_motorControllers[Constants.kDriveRightRearIndex].setFollow(m_motorControllers[Constants.kDriveRightFrontIndex]);
    m_motorControllers[Constants.kDriveRightMiddleIndex].setFollow(m_motorControllers[Constants.kDriveRightFrontIndex]);
  }

  @Override
  public void periodic() {
    // acradeDrive(); // with this method, delete drivebaseteleopcommand

    // Update the smart dashboard in here, runs a for loop so it does it for every motor
    for(int i = 0; i < m_motorControllers.length; i++) {
      m_motorControllers[i].updateSmartDashboard();
    }
 
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
    m_differentialDrive.arcadeDrive( m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), m_driverJoystick.getRawAxis(Constants.kDBRightJoystickAxisY));

  }

  // tank drive, not used but good to have
  public void tankDrive() {
    m_differentialDrive.tankDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), m_driverJoystick.getRawAxis(Constants.kDBRightJoystickAxisY));
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  public void arcadeDrive(double rotation) {
    m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), rotation);
  }

  @Override
  public void simulationPeriodic() {
    // Currently serves no purpose
  }

  public void driveFunction() {
    // currently serves no purpose
  }

  public void stopMotorsFunction() {
    // Calls Arcade Drive with a zero to both speed and rotation in order to stop the motors
    m_differentialDrive.arcadeDrive(0.0, 0.0);
  }


  // TODO: we can add more tankDrive co functions as extras later
}
