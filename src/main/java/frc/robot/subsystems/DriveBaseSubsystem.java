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

  private final Joystick driverJoystick;
  private final MotorController[] motorControllers = new MotorController[6];
  private final DifferentialDrive differentialDrive;

  
  public DriveBaseSubsystem(Joystick joystick) {  
    driverJoystick = joystick;
    
    differentialDrive = new DifferentialDrive(motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax(), motorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
    
    // motor controllers
    motorControllers[Constants.kDriveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.kDriveLeftFront);
    motorControllers[Constants.kDriveLeftMiddleIndex] = new MotorController("Differential Left Middle", Constants.kDriveLeftMiddle);
    motorControllers[Constants.kDriveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.kDriveLeftRear);
    motorControllers[Constants.kDriveRightFrontIndex] = new MotorController("Differential Right Front", Constants.kDriveRightFront);
    motorControllers[Constants.kDriveRightMiddleIndex] = new MotorController("Differential Right Middle", Constants.kDriveRightMiddle);
    motorControllers[Constants.kDriveRightRearIndex] = new MotorController("Differential Right Rear", Constants.kDriveRightRear);

    // inverses right side motors (2022 wpilib doesn't default it to be inverted for differential drive)
    motorControllers[Constants.kDriveRightFrontIndex].getSparkMax().setInverted(true);
    motorControllers[Constants.kDriveRightMiddleIndex].getSparkMax().setInverted(true);
    motorControllers[Constants.kDriveRightRearIndex].getSparkMax().setInverted(true);

    //Forces middle and rear motors of each side to follow the first
    motorControllers[Constants.kDriveLeftRearIndex].getSparkMax().follow(motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax());
    motorControllers[Constants.kDriveLeftMiddleIndex].getSparkMax().follow(motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax());
    motorControllers[Constants.kDriveRightRearIndex].getSparkMax().follow(motorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
    motorControllers[Constants.kDriveRightMiddleIndex].getSparkMax().follow(motorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
  }

  @Override
  public void periodic() {
    // Update the smart dashboard in here, runs a for loop so it does it for every motor
    for(int i = 0; i < motorControllers.length; i++) {
      motorControllers[i].updateSmartDashboard();
    }
 
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
    differentialDrive.arcadeDrive(driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), driverJoystick.getRawAxis(Constants.kDBRightJoystickAxisY));

  }

  // tank drive, not used but good to have
  public void tankDrive() {
    differentialDrive.tankDrive(driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), driverJoystick.getRawAxis(Constants.kDBRightJoystickAxisY));
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  public void arcadeDrive(double rotation) {
    differentialDrive.arcadeDrive(driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), rotation);
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
    differentialDrive.arcadeDrive(0.0, 0.0);
  }

  // TODO: return actual speeds
  public double getLeftSpeed() {
    return 0.0;
  }

  public double getRightSpeed() {
    return 0.0;
  }


  // TODO: we can add more tanrive co functions as extras later
}
