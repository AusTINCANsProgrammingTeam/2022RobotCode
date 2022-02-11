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
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private boolean usingExternal = false;

  
  public DriveBaseSubsystem(Joystick joystick, boolean usingExternal) {  
    m_driverJoystick = joystick;
    m_motorControllers = new MotorController[4];
    m_gyro = new AHRS(I2C.Port.kMXP);    

    // motor controllers
    m_motorControllers[Constants.driveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.driveLeftFront, Constants.driveBaseCurrentLimit, true);
    m_motorControllers[Constants.driveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.driveLeftRear, Constants.driveBaseCurrentLimit, true);
    m_motorControllers[Constants.driveRightFrontIndex] = new MotorController("Differential Right Front", Constants.driveRightFront, Constants.driveBaseCurrentLimit, true);
    m_motorControllers[Constants.driveRightRearIndex] = new MotorController("Differential Right Rear", Constants.driveRightRear, Constants.driveBaseCurrentLimit, true);

    // invert left side motors
    m_motorControllers[Constants.driveLeftFrontIndex].setInverted(true);
    m_motorControllers[Constants.driveLeftRearIndex].setInverted(true);

    //Forces middle and rear motors of each side to follow the first
    m_motorControllers[Constants.driveLeftRearIndex].setFollow(m_motorControllers[Constants.driveLeftFrontIndex]);
    m_motorControllers[Constants.driveRightRearIndex].setFollow(m_motorControllers[Constants.driveRightFrontIndex]);

    // differential drive
    m_differentialDrive = new DifferentialDrive(m_motorControllers[Constants.driveLeftFrontIndex].getSparkMax(), 
                                          m_motorControllers[Constants.driveRightFrontIndex].getSparkMax());

    this.usingExternal = usingExternal; // gets key if using external or internal encoders

    // // external encoders            
    // m_extLeftEncoder = new Encoder(Constants.leftEncoderDIOone, Constants.leftEncoderDIOtwo, 
    // false, Encoder.EncodingType.k2X); // TODO: confirm correct configuration for encoder
    // m_extRightEncoder = new Encoder(Constants.rightEncoderDIOone, Constants.rightEncoderDIOtwo, 
    // false, Encoder.EncodingType.k2X);

    // m_extLeftEncoder.setReverseDirection(true);
    
    // internal encoders
    m_internalLeftEncoder = m_motorControllers[Constants.driveLeftFrontIndex].getEncoder();
    m_internalRightEncoder = m_motorControllers[Constants.driveRightFrontIndex].getEncoder();

    // calculate circumference then convert to meters
    // wheel radius in inches, want to convert meters
    m_internalLeftEncoder.setPositionConversionFactor(Constants.gearRatio * 2 * Math.PI * Constants.wheelRadius / Constants.inchesInMeter); 
    m_internalRightEncoder.setPositionConversionFactor(Constants.gearRatio * 2 * Math.PI * Constants.wheelRadius / Constants.inchesInMeter);

    resetEncoders();
    
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d()); 

    // tuning PID, experimental values, multiply by 2
    m_motorControllers[Constants.driveLeftFrontIndex].getPID().setP(0.000005);
    m_motorControllers[Constants.driveLeftRearIndex].getPID().setP(0.000005);
    m_motorControllers[Constants.driveRightFrontIndex].getPID().setP(0.000005);
    m_motorControllers[Constants.driveRightRearIndex].getPID().setP(0.000005);

    m_motorControllers[Constants.driveLeftFrontIndex].getPID().setI(0.0000025);
    m_motorControllers[Constants.driveLeftRearIndex].getPID().setI(0.0000025);
    m_motorControllers[Constants.driveRightFrontIndex].getPID().setI(0.0000025);
    m_motorControllers[Constants.driveRightRearIndex].getPID().setI(0.0000025);
  }

  @Override
  public void periodic() {

    // update odometry

    // checks which encoders are being used: external or internal
    if(usingExternal) {
      m_odometry.update(
        m_gyro.getRotation2d(), m_extLeftEncoder.getDistance(), m_extRightEncoder.getDistance());
    }
    else {  // internal
      double leftPosition = m_internalLeftEncoder.getPosition();    // automatically applies conversion factor
      double rightPosition = m_internalRightEncoder.getPosition();

      m_odometry.update(
        m_gyro.getRotation2d(), leftPosition, rightPosition);
    }

    SmartDashboard.putNumber("Left IAccum", m_motorControllers[Constants.driveLeftFrontIndex].getPID().getIAccum());
    SmartDashboard.putNumber("Right IAccum", m_motorControllers[Constants.driveRightFrontIndex].getPID().getIAccum());

    // Update the smart dashboard in here, runs a for loop so it does it for every motor
    for(int i = 0; i < m_motorControllers.length; i++) {
      m_motorControllers[i].updateSmartDashboard();
    }
 
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
    m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), 
                                      m_driverJoystick.getRawAxis(Constants.DBRightJoystickAxisX));
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

  public void resetEncoders() {
    if(usingExternal) {
      m_extLeftEncoder.reset();
      m_extRightEncoder.reset();
    }
    else {
      m_internalLeftEncoder.setPosition(0);
      m_internalRightEncoder.setPosition(0);
    }
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
    // leftSpeed and rightSpeed in m/s, need to convert it to rpm

    leftSpeed = leftSpeed * Constants.inchesInMeter;  // meters to inches because radius is in inches
    rightSpeed = rightSpeed * Constants.inchesInMeter;

    leftSpeed = leftSpeed / Constants.wheelRadius;  // convert it to angular velocities
    rightSpeed = rightSpeed / Constants.wheelRadius;

    leftSpeed = leftSpeed / (2*Math.PI);    // convert it to rotation per second
    rightSpeed = rightSpeed / (2*Math.PI);

    leftSpeed = leftSpeed * Constants.gearRatio;    // apply gear ratio of 10.75 : 1 (motor to wheel)
    rightSpeed = rightSpeed * Constants.gearRatio;

    leftSpeed = leftSpeed * 60;   // convert it to rotation per minute
    rightSpeed = rightSpeed * 60;

    SmartDashboard.putNumber("left speed (rpm): ", leftSpeed);
    SmartDashboard.putNumber("right speed (rpm): ", rightSpeed);

    m_motorControllers[Constants.driveLeftFrontIndex].getPID().setReference(leftSpeed, CANSparkMax.ControlType.kVelocity);
    m_motorControllers[Constants.driveRightFrontIndex].getPID().setReference(rightSpeed, CANSparkMax.ControlType.kVelocity);
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

  public double[] getPositions() {
    double[] positions = new double[2];
    positions[0] = m_internalLeftEncoder.getPosition();
    positions[1] = m_internalRightEncoder.getPosition();
    return positions;
  }

  // TODO: we can add more tankdrive co functions as extras later
}
                                                                                                                                                                                              