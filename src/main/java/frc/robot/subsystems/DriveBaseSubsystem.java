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
    
    differentialDrive = new DifferentialDrive(motorControllers[Constants.driveLeftFrontIndex].getSparkMax(), motorControllers[Constants.driveRightFrontIndex].getSparkMax());
    
    // motor controllers
    motorControllers[Constants.driveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.driveLeftFront);
    motorControllers[Constants.driveLeftMiddleIndex] = new MotorController("Differential Left Middle", Constants.driveLeftMiddle);
    motorControllers[Constants.driveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.driveLeftRear);
    motorControllers[Constants.driveRightFrontIndex] = new MotorController("Differential Right Front", Constants.driveRightFront);
    motorControllers[Constants.driveRightMiddleIndex] = new MotorController("Differential Right Middle", Constants.driveRightMiddle);
    motorControllers[Constants.driveRightRearIndex] = new MotorController("Differential Right Rear", Constants.driveRightRear);

    // inverses right side motors (2022 wpilib doesn't default it to be inverted for differential drive)
    motorControllers[Constants.driveRightFrontIndex].getSparkMax().setInverted(true);
    motorControllers[Constants.driveRightMiddleIndex].getSparkMax().setInverted(true);
    motorControllers[Constants.driveRightRearIndex].getSparkMax().setInverted(true);

    //Forces middle and rear motors of each side to follow the first
    motorControllers[Constants.driveLeftRearIndex].getSparkMax().follow(motorControllers[Constants.driveLeftFrontIndex].getSparkMax());
    motorControllers[Constants.driveLeftMiddleIndex].getSparkMax().follow(motorControllers[Constants.driveLeftFrontIndex].getSparkMax());
    motorControllers[Constants.driveRightRearIndex].getSparkMax().follow(motorControllers[Constants.driveRightFrontIndex].getSparkMax());
    motorControllers[Constants.driveRightMiddleIndex].getSparkMax().follow(motorControllers[Constants.driveRightFrontIndex].getSparkMax());
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
    differentialDrive.arcadeDrive(driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), driverJoystick.getRawAxis(Constants.DBRightJoystickAxisY));

  }

  // tank drive, not used but good to have
  public void tankDrive() {
    differentialDrive.tankDrive(driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), driverJoystick.getRawAxis(Constants.DBRightJoystickAxisY));
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  public void arcadeDrive(double rotation) {
    differentialDrive.arcadeDrive(driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), rotation);
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
