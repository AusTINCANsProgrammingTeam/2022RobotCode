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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  private SimpleMotorFeedforward m_sMotorFeedforward;

  private boolean usingExternal = false;

  
  public DriveBaseSubsystem(Joystick joystick, boolean usingExternal) {  
    m_driverJoystick = joystick;
    m_motorControllers = new MotorController[4];
    m_gyro = new AHRS(I2C.Port.kMXP);
    m_gyro.reset(); // resets the heading of the robot to 0

    m_sMotorFeedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);

    // motor controllers
    m_motorControllers[Constants.driveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.driveLeftFront, Constants.driveBaseCurrentLimit, true);
    m_motorControllers[Constants.driveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.driveLeftRear, Constants.driveBaseCurrentLimit);
    m_motorControllers[Constants.driveRightFrontIndex] = new MotorController("Differential Right Front", Constants.driveRightFront, Constants.driveBaseCurrentLimit, true);
    m_motorControllers[Constants.driveRightRearIndex] = new MotorController("Differential Right Rear", Constants.driveRightRear, Constants.driveBaseCurrentLimit);

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
    //                          false, Encoder.EncodingType.k2X); // TODO: confirm correct configuration for encoder
    // m_extRightEncoder = new Encoder(Constants.rightEncoderDIOone, Constants.rightEncoderDIOtwo, 
    //                          false, Encoder.EncodingType.k2X);

    // m_extLeftEncoder.setReverseDirection(true);
    
    // internal encoders
    m_internalLeftEncoder = m_motorControllers[Constants.driveLeftFrontIndex].getEncoder();
    m_internalRightEncoder = m_motorControllers[Constants.driveRightFrontIndex].getEncoder();

    // calculate circumference then convert to meters
    // wheel radius in inches, want to convert meters
    // divide by gear ratio to get in terms of motor rotations when multiplied to number of motor rotations
    m_internalLeftEncoder.setPositionConversionFactor(
              2 * Math.PI * Constants.wheelRadius / Constants.inchesInMeter / Constants.gearRatio); 
    m_internalRightEncoder.setPositionConversionFactor(
              2 * Math.PI * Constants.wheelRadius / Constants.inchesInMeter / Constants.gearRatio);

    resetEncoders();  // reset encoders to reset position and velocity values
    
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // set PID values
    m_motorControllers[Constants.driveRightFrontIndex].getPID().setP(Constants.driveRightPID[0]);
    m_motorControllers[Constants.driveRightFrontIndex].getPID().setI(Constants.driveRightPID[1]);
    m_motorControllers[Constants.driveRightFrontIndex].getPID().setD(Constants.driveRightPID[2]);

    m_motorControllers[Constants.driveLeftFrontIndex].getPID().setP(Constants.driveLeftPID[0]);
    m_motorControllers[Constants.driveLeftFrontIndex].getPID().setI(Constants.driveLeftPID[1]);
    m_motorControllers[Constants.driveLeftFrontIndex].getPID().setD(Constants.driveLeftPID[2]);

    // rear motor pid controllers should follow
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

      SmartDashboard.putNumber("Left position", leftPosition);
      SmartDashboard.putNumber("Right position", rightPosition);

      m_odometry.update(
        m_gyro.getRotation2d(), leftPosition, rightPosition);
    }

    SmartDashboard.putNumber("Left IAccum", m_motorControllers[Constants.driveLeftFrontIndex].getPID().getIAccum());
    SmartDashboard.putNumber("Right IAccum", m_motorControllers[Constants.driveRightFrontIndex].getPID().getIAccum());

    // Update the smart dashboard in here
    // updates pid values of leaders only not the followers
    m_motorControllers[Constants.driveLeftFrontIndex].updateSmartDashboard();
    m_motorControllers[Constants.driveRightFrontIndex].updateSmartDashboard();
 
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
    m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.leftJoystickY), 
                                    m_driverJoystick.getRawAxis(Constants.rightJoystickX));
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  public void arcadeDrive(double rotation) {
    //m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), rotation);
  }

  //TODO: Make a command to switch modes (extra)

  // tank drive, not used but good to have
  public void tankDrive() {
    m_differentialDrive.tankDrive(m_driverJoystick.getRawAxis(Constants.leftJoystickY), 
                                  m_driverJoystick.getRawAxis(Constants.rightJoystickY));
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

    leftSpeed = leftSpeed * Constants.inchesInMeter;  // meters to inches to work with radius in inches
    rightSpeed = rightSpeed * Constants.inchesInMeter;


    // // set feedforward constrainty
    // m_motorControllers[Constants.driveLeftFrontIndex].getPID().setFF(m_sMotorFeedforward.calculate(leftSpeed));
    // m_motorControllers[Constants.driveLeftRearIndex].getPID().setFF(m_sMotorFeedforward.calculate(leftSpeed));

    // m_motorControllers[Constants.driveRightFrontIndex].getPID().setFF(m_sMotorFeedforward.calculate(rightSpeed));
    // m_motorControllers[Constants.driveRightRearIndex].getPID().setFF(m_sMotorFeedforward.calculate(rightSpeed));


    leftSpeed = leftSpeed / Constants.wheelRadius;  // convert it to angular velocity
    rightSpeed = rightSpeed / Constants.wheelRadius;

    leftSpeed = leftSpeed / (2*Math.PI);    // convert it to rotations per second, 1 rotation = 2pi radians
    rightSpeed = rightSpeed / (2*Math.PI);

    leftSpeed = leftSpeed * 60;   // convert it to rotations per minute (rpm)
    rightSpeed = rightSpeed * 60; 

    leftSpeed = leftSpeed * Constants.gearRatio;    // apply gear ratio of 10.75 motor rotations : 1 wheel rotation
    rightSpeed = rightSpeed * Constants.gearRatio;  // in wheel terms right now, 
                                                    // need to get into motor rotational terms to feed to internal pid

    SmartDashboard.putNumber("left speed (rpm) [biconsumer]", leftSpeed);
    SmartDashboard.putNumber("right speed (rpm [biconsumer])", rightSpeed);

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
                                                                                                                                                                                              