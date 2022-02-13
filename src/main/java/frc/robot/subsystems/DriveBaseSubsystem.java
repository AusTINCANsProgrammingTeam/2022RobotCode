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
import edu.wpi.first.wpilibj.Encoder;


public class DriveBaseSubsystem extends SubsystemBase {

  private final Joystick m_driverJoystick;
  private final MotorController[] m_motorControllers;
  private final DifferentialDrive m_differentialDrive;
  //public static ADIS16448_IMU m_gyro; Non-native gyro, might use later
  public static ADXRS450_Gyro m_gyro;
  private final DifferentialDriveOdometry m_odometry;
  public static Encoder m_leftEncoder;
  public static Encoder m_rightEncoder;

  // Here are the encoders
  
  

  public DriveBaseSubsystem(Joystick joystick) {  
    m_leftEncoder = new Encoder(Constants.leftEncoderDIOone, Constants.leftEncoderDIOtwo, 
    false, Encoder.EncodingType.k2X);
    m_rightEncoder = new Encoder(Constants.rightEncoderDIOone, Constants.rightEncoderDIOtwo, 
    false, Encoder.EncodingType.k2X);
    m_driverJoystick = joystick;
    m_motorControllers = new MotorController[4];
    m_gyro = new ADXRS450_Gyro();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    
    

    // motor controllers
    m_motorControllers[Constants.driveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.driveLeftFront);
    m_motorControllers[Constants.driveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.driveLeftRear);
    m_motorControllers[Constants.driveRightFrontIndex] = new MotorController("Differential Right Front", Constants.driveRightFront);
    m_motorControllers[Constants.driveRightRearIndex] = new MotorController("Differential Right Rear", Constants.driveRightRear);

    // inverses right side motors (2022 wpilib doesn't default it to be inverted for differential drive)
    m_motorControllers[Constants.driveRightFrontIndex].setInverted(true);
    m_motorControllers[Constants.driveRightRearIndex].setInverted(true);

    //Forces middle and rear motors of each side to follow the first
    m_motorControllers[Constants.driveLeftRearIndex].setFollow(m_motorControllers[Constants.driveLeftFrontIndex]);
    m_motorControllers[Constants.driveRightRearIndex].setFollow(m_motorControllers[Constants.driveRightFrontIndex]);

    // differential drive
    m_differentialDrive = new DifferentialDrive(m_motorControllers[Constants.driveLeftFrontIndex].getSparkMax(), 
                                          m_motorControllers[Constants.driveRightFrontIndex].getSparkMax());
  }

  @Override
  public void periodic() {
    arcadeDrive();  // periodically runs the arcadeDrive function, avoid scheduling the command

    // Update the smart dashboard in here, runs a for loop so it does it for every motor
    for(int i = 0; i < m_motorControllers.length; i++) {
      m_motorControllers[i].updateSmartDashboard();
    }
 
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
 //   m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), 
  //                                    m_driverJoystick.getRawAxis(Constants.DBRightJoystickAxisX));
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  //TODO: Make a command to switch modes (only if we actually want this)
  public void arcadeDrive(double rotation) {
    //m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), rotation);
  }

  // tank drive, not used but good to have
  public void tankDrive() {
    m_differentialDrive.tankDrive(m_driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), 
                                  m_driverJoystick.getRawAxis(Constants.DBRightJoystickAxisY));
  }

  public void setAutonVolts(double leftVolts, double rightVolts) {
    m_motorControllers[Constants.driveLeftFrontIndex].getSparkMax().setVoltage(leftVolts);
    m_motorControllers[Constants.driveRightFrontIndex].getSparkMax().setVoltage(rightVolts);
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

  public CANSparkMax getRightMotor() {
    return m_motorControllers[Constants.driveRightFrontIndex].getSparkMax();
  }

  public CANSparkMax getLeftMotor() {
    return m_motorControllers[Constants.driveLeftFrontIndex].getSparkMax();
  }

  // return speed of left side motors
  public double getLeftSpeed() {
    return m_motorControllers[Constants.driveLeftFrontIndex].getSpeed();
  }

  // return speed of right side motors
  public double getRightSpeed() {
    return m_motorControllers[Constants.driveRightFrontIndex].getSpeed();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();  // reset encoders
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void setSpeeds(double left, double right){
    getLeftMotor().set(left);
    getRightMotor().set(right);
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }
  

  // TODO: we can add more tankdrive co functions as extras later
}
