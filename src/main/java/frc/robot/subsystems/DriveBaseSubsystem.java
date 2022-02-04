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

  private Joystick m_driverJoystick;
  private MotorController[] m_motorControllers;
  private DifferentialDrive m_differentialDrive;
  private AHRS m_gyro;
  private DifferentialDriveOdometry m_odometry;

  // external encoders
  private Encoder m_extRightEncoder;
  private Encoder m_extLeftEncoder;

  // internal encoders
  private RelativeEncoder m_internalLeftEncoder;
  private RelativeEncoder m_internalRightEncoder;

  
  public DriveBaseSubsystem(Joystick joystick) {  
    m_driverJoystick = joystick;
    m_motorControllers = new MotorController[4];
    m_gyro = new AHRS(I2C.Port.kMXP);
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d()); 
    

    // motor controllers
    m_motorControllers[Constants.driveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.driveLeftFront);
    m_motorControllers[Constants.driveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.driveLeftRear);
    m_motorControllers[Constants.driveRightFrontIndex] = new MotorController("Differential Right Front", Constants.driveRightFront);
    m_motorControllers[Constants.driveRightRearIndex] = new MotorController("Differential Right Rear", Constants.driveRightRear);

    // invert left side motors
    m_motorControllers[Constants.driveLeftFrontIndex].setInverted(true);
    m_motorControllers[Constants.driveLeftRearIndex].setInverted(true);

    //Forces middle and rear motors of each side to follow the first
    m_motorControllers[Constants.driveLeftRearIndex].setFollow(m_motorControllers[Constants.driveLeftFrontIndex]);
    m_motorControllers[Constants.driveRightRearIndex].setFollow(m_motorControllers[Constants.driveRightFrontIndex]);

    // differential drive
    m_differentialDrive = new DifferentialDrive(m_motorControllers[Constants.driveLeftFrontIndex].getSparkMax(), 
                                          m_motorControllers[Constants.driveRightFrontIndex].getSparkMax());

    // external encoders            
    m_extLeftEncoder = new Encoder(Constants.leftEncoderDIOone, Constants.leftEncoderDIOtwo, 
    false, Encoder.EncodingType.k2X);
    m_extRightEncoder = new Encoder(Constants.rightEncoderDIOone, Constants.rightEncoderDIOtwo, 
    false, Encoder.EncodingType.k2X);
    
    // internal encoders
    m_internalLeftEncoder = m_motorControllers[Constants.kDriveLeftFrontIndex].getEncoder();
    m_internalRightEncoder = m_motorControllers[Constants.kDriveRightFrontIndex].getEncoder();  
  }

  @Override
  public void periodic() {

    // update odometry
    m_odometry.update(
        m_gyro.getRotation2d(), m_extLeftEncoder.getDistance(), m_extRightEncoder.getDistance());

    // Update the smart dashboard in here, runs a for loop so it does it for every motor
    for(int i = 0; i < m_motorControllers.length; i++) {
      m_motorControllers[i].updateSmartDashboard();
    }
 
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
    m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), 
                                      m_driverJoystick.getRawAxis(Constants.DBRightJoystickAxisY));
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  public void arcadeDrive(double rotation) {
    //m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), rotation);
  }

  //TODO: Make a command to switch modes (extra)

  // tank drive, not used but good to have
  public void tankDrive() {
    m_differentialDrive.tankDrive(m_driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), 
                                  m_driverJoystick.getRawAxis(Constants.DBRightJoystickAxisY));
  }

  // public void setAutonVolts(double leftVolts, double rightVolts) {
  //   m_motorControllers[Constants.driveLeftFrontIndex].getSparkMax().setVoltage(leftVolts);
  //   m_motorControllers[Constants.driveRightFrontIndex].getSparkMax().setVoltage(rightVolts);
  //   m_differentialDrive.feed();
  // }

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
    return m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax();
  }

  public CANSparkMax getLeftMotor() {
    return m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax();
  }

  // return speed of left side motors
  public double getLeftSpeed() {
    return m_motorControllers[Constants.driveLeftFrontIndex].getSpeed();
  }

  // return speed of right side motors
  public double getRightSpeed() {
    return m_motorControllers[Constants.driveRightFrontIndex].getSpeed();
  }

  public void resetEncoders() {
    m_extLeftEncoder.reset();
    m_extRightEncoder.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();  // reset encoders
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setSpeeds(double left, double right){
    getLeftMotor().set(left);
    getRightMotor().set(right);
  }

  // for trajectory (ramseteCommand)
  public void acceptWheelSpeeds(double leftSpeed, double rightSpeed) {
    m_motorControllers[Constants.driveLeftFrontIndex].setSpeed(leftSpeed);
    m_motorControllers[Constants.driveRightFrontIndex].setSpeed(rightSpeed);
    m_differentialDrive.feed();
  }
  
  // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  //   return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  // }


  // for debug/shuffleboard

  // return gyro info
  public double getGyroAngle() {
    return m_gyro.getAngle();
  }
  
  // returns velocity conversion factor from the internal encoder in order to calculate distances and speeds, avoiding the use of external encoders
  public double getVelocityConversionFactor() {
    return m_internalRightEncoder.getVelocityConversionFactor();
  }

  // TODO: we can add more tankdrive co functions as extras later
}
