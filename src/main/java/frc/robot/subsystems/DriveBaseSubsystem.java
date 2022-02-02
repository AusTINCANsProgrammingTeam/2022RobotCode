// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;


public class DriveBaseSubsystem extends SubsystemBase {

  private final Joystick m_driverJoystick;
  private final MotorController[] m_motorControllers;
  private final DifferentialDrive m_differentialDrive;
  //public static ADIS16448_IMU m_gyro; //Non-native gyro, might use later
  //public static ADXRS450_Gyro m_gyro;
  private AHRS m_gyro;
  private final DifferentialDriveOdometry m_odometry;
  public static RelativeEncoder m_leftEncoder;
  public static RelativeEncoder m_rightEncoder;

  
  public DriveBaseSubsystem(Joystick joystick) {  
    m_driverJoystick = joystick;
    m_motorControllers = new MotorController[4];
    //m_gyro = new ADIS16448_IMU();
    m_gyro = new AHRS(I2C.Port.kMXP);
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d()); 
    

    // motor controllers
    m_motorControllers[Constants.kDriveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.kDriveLeftFront, 40, true);
    m_motorControllers[Constants.kDriveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.kDriveLeftRear, 40, true);
    m_motorControllers[Constants.kDriveRightFrontIndex] = new MotorController("Differential Right Front", Constants.kDriveRightFront, 40, true);
    m_motorControllers[Constants.kDriveRightRearIndex] = new MotorController("Differential Right Rear", Constants.kDriveRightRear, 40, true);

    // inverses right side motors (2022 wpilib doesn't default it to be inverted for differential drive)
    m_motorControllers[Constants.kDriveRightFrontIndex].setInverted(true);
    m_motorControllers[Constants.kDriveRightRearIndex].setInverted(true);

    //Forces middle and rear motors of each side to follow the first
    m_motorControllers[Constants.kDriveLeftRearIndex].setFollow(m_motorControllers[Constants.kDriveLeftFrontIndex]);
    m_motorControllers[Constants.kDriveRightRearIndex].setFollow(m_motorControllers[Constants.kDriveRightFrontIndex]);
    
    // Encoders
    m_leftEncoder = m_motorControllers[Constants.kDriveLeftFrontIndex].getEncoder();
    m_rightEncoder = m_motorControllers[Constants.kDriveRightFrontIndex].getEncoder();  
    // differential drive
    m_differentialDrive = new DifferentialDrive(m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax(), 
                                          m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
  }

  @Override
  public void periodic() {
    //arcadeDrive();  // periodically runs the arcadeDrive function, avoid scheduling the command

    // Update the smart dashboard in here, runs a for loop so it does it for every motor
    for(int i = 0; i < m_motorControllers.length; i++) {
      m_motorControllers[i].updateSmartDashboard();
    }
 
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
    m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), 
                                      m_driverJoystick.getRawAxis(Constants.kDBRightJoystickAxisY));
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  //TODO: Make a command to switch modes (only if we actually want this)
  public void arcadeDrive(double rotation) {
    //m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), rotation);
  }

  // tank drive, not used but good to have
  public void tankDrive() {
    m_differentialDrive.tankDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), 
                                  m_driverJoystick.getRawAxis(Constants.kDBRightJoystickAxisY));
  }

  public void setAutonVolts(double leftVolts, double rightVolts) {
    m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax().setVoltage(leftVolts);
    m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax().setVoltage(rightVolts);
    m_differentialDrive.feed();
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

  // public CANSparkMax getRightMotor() {
  //   return m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax();
  // }

  // public CANSparkMax getLeftMotor() {
  //   return m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax();
  // }

  // return speed of left side motors
  public double getLeftSpeed() {
    return m_motorControllers[Constants.kDriveLeftFrontIndex].getSpeed();
  }

  // return speed of right side motors
  public double getRightSpeed() {
    return m_motorControllers[Constants.kDriveRightFrontIndex].getSpeed();
  }
  // return gyro info
  public double getGyroAngle() {
    return m_gyro.getAngle();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();  // reset encoders
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }
  
  public double getVelocityConversionFactor() {
    return m_rightEncoder.getVelocityConversionFactor();
  }

  // TODO: we can add more tankdrive co functions as extras later
}
