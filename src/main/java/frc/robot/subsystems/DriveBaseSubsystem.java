// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.common.hardware.MotorController;

public class DriveBaseSubsystem extends SubsystemBase {
  private ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Teleop tab");
  private NetworkTableEntry teleopSpeed =
      driverTab.add("Speed percentage", 100).withPosition(0, 1).getEntry();

  private final Joystick m_driverJoystick;
  private final MotorController[] m_motorControllers;
  private final DifferentialDrive m_differentialDrive;
  private DifferentialDrivetrainSim m_DifferentialDrivetrainSim;
  public final Field2d m_field = new Field2d();
  private AHRS m_gyro;
  private AnalogGyroSim m_gyroSim;
  public static ADIS16448_IMU m_gyro2; // Non-native gyro, might use later
  private AnalogGyro m_gyro1;
  private final DifferentialDriveOdometry m_odometry;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;

  // Encoders for Sim
  private Encoder m_LEncoderForSim;
  private Encoder m_REncoderForSim;

  // internal encoders
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  private SimpleMotorFeedforward m_sMotorFeedforward;

  private boolean usingExternal = false;

  public DriveBaseSubsystem(Joystick joystick, boolean usingExternal) {

    m_driverJoystick = joystick;

    m_motorControllers = new MotorController[4];
    m_gyro = new AHRS(I2C.Port.kMXP);
    // m_gyro.reset(); // resets the heading of the robot to 0
    // m_gyro1 = new AnalogGyro(1);

    if (Robot.isSimulation()) {
      if (!usingExternal) {
        m_LEncoderForSim = new Encoder(1, 2);
        m_REncoderForSim = new Encoder(3, 4);
      }
    }

    m_sMotorFeedforward =
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter);

    // motor controllers
    m_motorControllers[Constants.driveLeftFrontIndex] =
        new MotorController(
            "Differential Left Front",
            Constants.driveLeftFront,
            Constants.driveBaseCurrentLimit,
            true);
    m_motorControllers[Constants.driveLeftRearIndex] =
        new MotorController(
            "Differential Left Rear", Constants.driveLeftRear, Constants.driveBaseCurrentLimit);
    m_motorControllers[Constants.driveRightFrontIndex] =
        new MotorController(
            "Differential Right Front",
            Constants.driveRightFront,
            Constants.driveBaseCurrentLimit,
            true);
    m_motorControllers[Constants.driveRightRearIndex] =
        new MotorController(
            "Differential Right Rear", Constants.driveRightRear, Constants.driveBaseCurrentLimit);

    // invert right side motors
    m_motorControllers[Constants.driveRightFrontIndex].setInverted(true);
    m_motorControllers[Constants.driveRightRearIndex].setInverted(true);

    // Forces rear motors of each side to follow the first
    m_motorControllers[Constants.driveLeftRearIndex].follow(
        m_motorControllers[Constants.driveLeftFrontIndex]);
    m_motorControllers[Constants.driveRightRearIndex].follow(
        m_motorControllers[Constants.driveRightFrontIndex]);

    // open loop ramp rate
    // m_motorControllers[Constants.driveLeftFrontIndex].setOpenLoopRampRate(.1);
    // m_motorControllers[Constants.driveRightFrontIndex].setOpenLoopRampRate(.1);
    // m_motorControllers[Constants.driveLeftRearIndex].setOpenLoopRampRate(.1);
    // m_motorControllers[Constants.driveRightRearIndex].setOpenLoopRampRate(.1);

    // differential drive
    m_differentialDrive =
        new DifferentialDrive(
            m_motorControllers[Constants.driveLeftFrontIndex],
            m_motorControllers[Constants.driveRightFrontIndex]);

    this.usingExternal = usingExternal; // gets key if using external or internal encoders
    initializeEncoders();

    if (Robot.isSimulation()) {
      m_gyro1 = new AnalogGyro(1);
      m_gyroSim = new AnalogGyroSim(m_gyro1);
      if (usingExternal == true) {
        // m_leftEncoderSim = new EncoderSim(m_leftEncoder);   // no "Encoder" object anymore
        // m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      } else {
        m_leftEncoderSim = new EncoderSim(m_LEncoderForSim);
        m_rightEncoderSim = new EncoderSim(m_REncoderForSim);
      }

      SmartDashboard.putData("Field", m_field);

      m_DifferentialDrivetrainSim =
          new DifferentialDrivetrainSim(
              DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
              7.29, // 7.29:1 gearing reduction.
              10, // MOI of 7.5 kg m^2 (from CAD model).
              60.0, // mass of the robot
              Units.inchesToMeters(3), // The robot wheel radius
              1, // The track width
              VecBuilder.fill(
                  0.001, 0.001, 0.001, 0.1, 0.1, 0.005,
                  0.005)); // standard deviations for measurement noise: x and y: 0.001m heading:
      // 0.001 rad  l and r velocity: 0.1m/s  l and r position: 0.005m
    }

    resetEncoders(); // reset encoders to reset position and velocity values

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // set PID values
    m_motorControllers[Constants.driveRightFrontIndex]
        .getPIDCtrl()
        .setP(Constants.driveRightPID[0]);
    m_motorControllers[Constants.driveRightFrontIndex]
        .getPIDCtrl()
        .setI(Constants.driveRightPID[1]);
    m_motorControllers[Constants.driveRightFrontIndex]
        .getPIDCtrl()
        .setD(Constants.driveRightPID[2]);

    m_motorControllers[Constants.driveLeftFrontIndex].getPIDCtrl().setP(Constants.driveLeftPID[0]);
    m_motorControllers[Constants.driveLeftFrontIndex].getPIDCtrl().setI(Constants.driveLeftPID[1]);
    m_motorControllers[Constants.driveLeftFrontIndex].getPIDCtrl().setD(Constants.driveLeftPID[2]);

    // rear motor pid controllers should follow
  }

  private void initializeEncoders() {
    if (usingExternal) {
      // external encoders
      m_leftEncoder =
          m_motorControllers[Constants.driveLeftFrontIndex].getAlternateEncoder(
              Constants.encoderCountsPerRev);
      m_rightEncoder =
          m_motorControllers[Constants.driveRightFrontIndex].getAlternateEncoder(
              Constants.encoderCountsPerRev);

    } else {
      // internal encoders
      m_leftEncoder = m_motorControllers[Constants.driveLeftFrontIndex].getEncoder();
      m_rightEncoder = m_motorControllers[Constants.driveRightFrontIndex].getEncoder();
      // no need to invert internal encoders, automatic

      // calculate circumference then convert to meters
      // wheel radius in inches, want to convert meters
      // divide by gear ratio to get in terms of motor rotations when multiplied to number of motor
      // rotations
      m_leftEncoder.setPositionConversionFactor(
          2 * Math.PI * Constants.wheelRadius / Constants.inchesInMeter / Constants.gearRatio);
      m_rightEncoder.setPositionConversionFactor(
          2 * Math.PI * Constants.wheelRadius / Constants.inchesInMeter / Constants.gearRatio);
    }
  }

  @Override
  public void periodic() {
    // update odometryreset
    double leftPosition = m_leftEncoder.getPosition();
    double rightPosition = m_rightEncoder.getPosition();
    m_odometry.update(m_gyro.getRotation2d(), leftPosition, rightPosition);

    // Update the smart dashboard here

    // updates pid values of leaders only not the followers
    m_motorControllers[Constants.driveLeftFrontIndex].updateSmartDashboard();
    m_motorControllers[Constants.driveRightFrontIndex].updateSmartDashboard();
  }

  public void setArcadedrivespeed(double input) {
    teleopSpeed.setDouble(input);
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
    m_differentialDrive.arcadeDrive(
        m_driverJoystick.getRawAxis(Constants.leftJoystickY) * (teleopSpeed.getDouble(100) / 100),
        -0.85 * m_driverJoystick.getRawAxis(Constants.rightJoystickX),
        true);
    // joystick has y-axis flipped so up is negative why down is positive
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  public void arcadeDrive(double rotation) {
    m_differentialDrive.arcadeDrive(
        -1 * m_driverJoystick.getRawAxis(Constants.leftJoystickY), rotation);
  }

  // TODO: Make a command to switch modes (extra)

  // tank drive, not used but good to have
  // TODO: check tankdrive if joystick axes are working
  public void tankDrive() {
    m_differentialDrive.tankDrive(
        -1 * m_driverJoystick.getRawAxis(Constants.leftJoystickY),
        -1 * m_driverJoystick.getRawAxis(Constants.rightJoystickY));
  }

  @Override
  public void simulationPeriodic() {
    m_DifferentialDrivetrainSim.setInputs(
        -m_motorControllers[Constants.driveLeftFrontIndex].get()
            * RobotController.getInputVoltage(),
        -m_motorControllers[Constants.driveRightFrontIndex].get()
            * RobotController.getInputVoltage());

    m_gyroSim.setAngle(-m_DifferentialDrivetrainSim.getHeading().getDegrees());

    m_DifferentialDrivetrainSim.update(0.01);

    SmartDashboard.putNumber("LMotor", m_motorControllers[Constants.driveLeftFrontIndex].get());
    SmartDashboard.putNumber("RMotor", m_motorControllers[Constants.driveRightFrontIndex].get());

    m_leftEncoderSim.setDistance(m_DifferentialDrivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_DifferentialDrivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_DifferentialDrivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_DifferentialDrivetrainSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_DifferentialDrivetrainSim.getHeading().getDegrees());

    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoderSim.getDistance(), m_rightEncoderSim.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  public void stopMotorsFunction() {
    // Calls Arcade Drive with a zero to both speed and rotation in order to stop the motors
    m_differentialDrive.arcadeDrive(0.0, 0.0);
  }

  public CANSparkMax getRightMotor() {
    return m_motorControllers[Constants.driveRightFrontIndex];
  }

  public CANSparkMax getLeftMotor() {
    return m_motorControllers[Constants.driveLeftFrontIndex];
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
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders(); // reset encoders
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setSpeeds(double left, double right) {
    getLeftMotor().set(left);
    getRightMotor().set(right);
  }

  // for trajectory (ramseteCommand)
  public void acceptWheelSpeeds(double leftSpeed, double rightSpeed) {
    // leftSpeed and rightSpeed in m/s, need to convert it to rpm

    leftSpeed =
        leftSpeed * Constants.inchesInMeter; // meters to inches to work with radius in inches
    rightSpeed = rightSpeed * Constants.inchesInMeter;

    leftSpeed = leftSpeed / Constants.wheelRadius; // convert it to angular velocity
    rightSpeed = rightSpeed / Constants.wheelRadius;

    leftSpeed =
        leftSpeed / (2 * Math.PI); // convert it to rotations per second, 1 rotation = 2pi radians
    rightSpeed = rightSpeed / (2 * Math.PI);

    leftSpeed = leftSpeed * 60; // convert it to rotations per minute (rpm)
    rightSpeed = rightSpeed * 60;

    leftSpeed =
        leftSpeed
            * Constants.gearRatio; // apply gear ratio of 10.75 motor rotations : 1 wheel rotation
    rightSpeed = rightSpeed * Constants.gearRatio; // in wheel terms right now,
    // need to get into motor rotational terms to feed to internal pid

    SmartDashboard.putNumber("left speed (rpm) [biconsumer]", leftSpeed);
    SmartDashboard.putNumber("right speed (rpm [biconsumer])", rightSpeed);

    m_motorControllers[Constants.driveLeftFrontIndex]
        .getPIDCtrl()
        .setReference(leftSpeed, CANSparkMax.ControlType.kVelocity, 0, Constants.arbFeedForward);
    m_motorControllers[Constants.driveRightFrontIndex]
        .getPIDCtrl()
        .setReference(rightSpeed, CANSparkMax.ControlType.kVelocity, 0, Constants.arbFeedForward);

    // m_motorControllers[Constants.driveLeftFrontIndex].getPID().setReference(leftSpeed,
    // CANSparkMax.ControlType.kVelocity);
    // m_motorControllers[Constants.driveRightFrontIndex].getPID().setReference(rightSpeed,
    // CANSparkMax.ControlType.kVelocity);

    m_differentialDrive.feed();
  }

  // for debug/shuffleboard

  // return gyro
  public AHRS getGyro() {
    return m_gyro;
  }

  public double[] getPositions() {
    double[] positions = new double[2];
    positions[0] = m_leftEncoder.getPosition(); // in meters
    positions[1] = m_rightEncoder.getPosition();

    return positions;
  }
}
