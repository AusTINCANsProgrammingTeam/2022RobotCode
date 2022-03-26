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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort.Port;
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
  private double driveBaseSpeed;
  private final Joystick driverJoystick;
  private final MotorController[] motorControllers;
  private final DifferentialDrive differentialDrive;
  private DifferentialDrivetrainSim differentialDrivetrainSim;
  public final Field2d m_field = new Field2d();

  private AnalogGyroSim gyroSim;
  public static ADIS16448_IMU gyro2; // Non-native gyro, might use later
  private AnalogGyro gyro1;

  // Encoders for Sim
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  private Encoder leftEncoderForSim;
  private Encoder rightEncoderForSim;

  // internal encoders
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private SimpleMotorFeedforward simpleMotorFF;

  private final DifferentialDriveOdometry odometry;
  private AHRS gyro;

  private boolean usingExternal;

  // shuffleboard
  // each box on the shuffleboard, sb stands for shuffleboard
  private ShuffleboardTab dtTab;
  private NetworkTableEntry sbLeftEncoderSpeed;
  private NetworkTableEntry sbRightEncoderSpeed;
  private NetworkTableEntry sbLeftBiconsumerSpeed;
  private NetworkTableEntry sbRightBiconsumerSpeed;
  private NetworkTableEntry sbLeftPosition;
  private NetworkTableEntry sbRightPosition;

  public DriveBaseSubsystem(Joystick joystick, boolean usingExternal) {
    driveBaseSpeed = 1;
    driverJoystick = joystick;

    motorControllers = new MotorController[4];
    gyro = new AHRS(Port.kMXP);
    gyro.reset(); // resets the heading of the robot to 0

    this.usingExternal = usingExternal;

    if (Robot.isSimulation()) {
      if (!usingExternal) {
        leftEncoderForSim = new Encoder(1, 2);
        rightEncoderForSim = new Encoder(3, 4);
      }
    }

    simpleMotorFF =
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants
                .kaVoltSecondsSquaredPerMeter); // We have this just in case we need it, not used at
    // the moment

    // motor controllers
    motorControllers[Constants.driveLeftFrontIndex] =
        new MotorController(
            "DriveBase Left Front", Constants.driveLeftFront, Constants.driveLeftPID);
    motorControllers[Constants.driveLeftRearIndex] =
        new MotorController("DriveBase Left Rear", Constants.driveLeftRear, Constants.driveLeftPID);
    motorControllers[Constants.driveRightFrontIndex] =
        new MotorController(
            "DriveBase Right Front", Constants.driveRightFront, Constants.driveRightPID);
    motorControllers[Constants.driveRightRearIndex] =
        new MotorController(
            "DriveBase Right Rear", Constants.driveRightRear, Constants.driveRightPID);

    // invert right side motors
    motorControllers[Constants.driveLeftFrontIndex].setInverted(true);
    motorControllers[Constants.driveLeftRearIndex].setInverted(true);

    motorControllers[Constants.driveLeftRearIndex].setOpenLoopRampRate(Constants.openLoopRampRate);
    motorControllers[Constants.driveLeftFrontIndex].setOpenLoopRampRate(Constants.openLoopRampRate);
    motorControllers[Constants.driveRightRearIndex].setOpenLoopRampRate(Constants.openLoopRampRate);
    motorControllers[Constants.driveRightFrontIndex].setOpenLoopRampRate(
        Constants.openLoopRampRate);

    motorControllers[Constants.driveLeftRearIndex].setSmartCurrentLimit(
        Constants.driveBaseCurrentLimit);
    motorControllers[Constants.driveLeftFrontIndex].setSmartCurrentLimit(
        Constants.driveBaseCurrentLimit);
    motorControllers[Constants.driveRightRearIndex].setSmartCurrentLimit(
        Constants.driveBaseCurrentLimit);
    motorControllers[Constants.driveRightFrontIndex].setSmartCurrentLimit(
        Constants.driveBaseCurrentLimit);

    // Forces rear motors of each side to follow the first
    motorControllers[Constants.driveLeftRearIndex].follow(
        motorControllers[Constants.driveLeftFrontIndex]);
    motorControllers[Constants.driveRightRearIndex].follow(
        motorControllers[Constants.driveRightFrontIndex]);

    // differential drive
    differentialDrive =
        new DifferentialDrive(
            motorControllers[Constants.driveLeftFrontIndex],
            motorControllers[Constants.driveRightFrontIndex]);

    this.usingExternal = usingExternal; // gets key if using external or internal encoders
    initializeEncoders();

    if (Robot.isSimulation()) {
      gyro1 = new AnalogGyro(1);
      gyroSim = new AnalogGyroSim(gyro1);
      if (usingExternal == true) {
        // leftEncoderSim = new EncoderSim); // no "Encoder" object
        // anymore
        // rightEncoderSim = new EncoderSim);
      } else {
        leftEncoderSim = new EncoderSim(leftEncoderForSim);
        rightEncoderSim = new EncoderSim(rightEncoderForSim);
      }

      SmartDashboard.putData("Field", m_field);

      differentialDrivetrainSim =
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
      // 0.001 rad l and r velocity: 0.1m/s l and r position: 0.005m
    }

    resetEncoders(); // reset encoders to reset position and velocity values

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    if (Constants.DebugMode) {
      initShuffleboard();
    }
  }

  private void initShuffleboard() {
    // gets the DriveBase tab, if it doesn't exist, create it with the name "DriveBase"
    dtTab = Shuffleboard.getTab("DriveBase");

    sbLeftEncoderSpeed =
        dtTab.add("Left Encoder Speed", 0).withSize(2, 2).withPosition(0, 0).getEntry();
    sbRightEncoderSpeed =
        dtTab.add("Right Encoder Speed", 0).withSize(2, 2).withPosition(4, 0).getEntry();

    sbLeftBiconsumerSpeed =
        dtTab.add("Left Biconsumer Speed", 0).withSize(2, 2).withPosition(0, 2).getEntry();
    sbRightBiconsumerSpeed =
        dtTab.add("Right Biconsumer Speed", 0).withSize(2, 2).withPosition(4, 2).getEntry();

    dtTab.add(gyro).withPosition(2, 0); // adds a gyro compass indicator

    sbLeftPosition = dtTab.add("Left Position", 0).withSize(2, 1).withPosition(0, 4).getEntry();
    sbRightPosition = dtTab.add("Right Position", 0).withSize(2, 1).withPosition(4, 4).getEntry();
  }

  private void initializeEncoders() {
    if (usingExternal) {
      // external encoders
      leftEncoder = getLeftMotor().getAlternateEncoder(Constants.encoderCountsPerRev);
      rightEncoder = getRightMotor().getAlternateEncoder(Constants.encoderCountsPerRev);

    } else {
      // internal encoders
      leftEncoder = getLeftMotor().getEncoder();
      rightEncoder = getRightMotor().getEncoder();
      // no need to invert internal encoders, automatic

      // calculate circumference then convert to meters
      // wheel radius in inches, want to convert meters
      // divide by gear ratio to get in terms of motor rotations when multiplied to
      // number of motor
      // rotations
      leftEncoder.setPositionConversionFactor(
          2 * Math.PI * Constants.wheelRadius / Constants.inchesInMeter / Constants.gearRatio);
      rightEncoder.setPositionConversionFactor(
          2 * Math.PI * Constants.wheelRadius / Constants.inchesInMeter / Constants.gearRatio);
    }
  }

  @Override
  public void periodic() {
    // update odometryreset
    double leftPosition = leftEncoder.getPosition();
    double rightPosition = rightEncoder.getPosition();
    odometry.update(gyro.getRotation2d(), leftPosition, rightPosition);

    // update shuffleboard
    if (Constants.DebugMode) {
      sbLeftEncoderSpeed.setDouble(leftEncoder.getVelocity());
      sbRightEncoderSpeed.setDouble(rightEncoder.getVelocity());
      sbLeftPosition.setDouble(leftEncoder.getPosition()); // in meters
      sbRightPosition.setDouble(rightEncoder.getPosition());
    }

    // Update the smart dashboard here
    // updates pid values of leaders, followers not needed
    motorControllers[Constants.driveLeftFrontIndex].updateSmartDashboard();
    motorControllers[Constants.driveRightFrontIndex].updateSmartDashboard();

    // for tuning pid on each wheel, TODO: remove when done
    // acceptWheelSpeeds(10, 0);
  }

  public void setDriveBaseSpeed(double driveBaseSpeed) {
    this.driveBaseSpeed = driveBaseSpeed;
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
    differentialDrive.arcadeDrive(
        driverJoystick.getRawAxis(Constants.leftJoystickY) * -driveBaseSpeed,
        driverJoystick.getRawAxis(Constants.rightJoystickX) * Constants.driveBaseTurnRate,
        true);
    // joystick has y-axis flipped so up is negative, multiply by negative to accomodate this
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  public void arcadeDrive(double rotation) {
    differentialDrive.arcadeDrive(
        driverJoystick.getRawAxis(Constants.leftJoystickY) * -driveBaseSpeed, rotation);
  }

  // TODO: Make a command to switch modes (extra)

  // tank drive, not used but good to have
  // TODO: check tankdrive if joystick axes are working
  public void tankDrive() {
    differentialDrive.tankDrive(
        -1 * driverJoystick.getRawAxis(Constants.leftJoystickY),
        -1 * driverJoystick.getRawAxis(Constants.rightJoystickY));
  }

  @Override
  public void simulationPeriodic() {
    differentialDrivetrainSim.setInputs(
        -motorControllers[Constants.driveLeftFrontIndex].get() * RobotController.getInputVoltage(),
        -motorControllers[Constants.driveRightFrontIndex].get()
            * RobotController.getInputVoltage());

    gyroSim.setAngle(-differentialDrivetrainSim.getHeading().getDegrees());

    differentialDrivetrainSim.update(0.01);

    SmartDashboard.putNumber("LMotor", motorControllers[Constants.driveLeftFrontIndex].get());
    SmartDashboard.putNumber("RMotor", motorControllers[Constants.driveRightFrontIndex].get());

    leftEncoderSim.setDistance(differentialDrivetrainSim.getLeftPositionMeters());
    leftEncoderSim.setRate(differentialDrivetrainSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(differentialDrivetrainSim.getRightPositionMeters());
    rightEncoderSim.setRate(differentialDrivetrainSim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-differentialDrivetrainSim.getHeading().getDegrees());

    odometry.update(
        gyro.getRotation2d(), leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
    m_field.setRobotPose(odometry.getPoseMeters());
  }

  public void stopDriveMotors() {
    // sets a velocity of 0 to each motor
    setSpeeds(0.0, 0.0);
  }

  private CANSparkMax getRightMotor() {
    return motorControllers[Constants.driveRightFrontIndex];
  }

  private CANSparkMax getLeftMotor() {
    return motorControllers[Constants.driveLeftFrontIndex];
  }

  private void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders(); // reset encoders
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
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

    if (Constants.DebugMode) {
      sbLeftBiconsumerSpeed.setDouble(leftSpeed);
      sbRightBiconsumerSpeed.setDouble(rightSpeed);
    }

    getLeftMotor()
        .getPIDController()
        .setReference(leftSpeed, CANSparkMax.ControlType.kVelocity, 0, Constants.arbFeedForward);
    getRightMotor()
        .getPIDController()
        .setReference(rightSpeed, CANSparkMax.ControlType.kVelocity, 0, Constants.arbFeedForward);

    differentialDrive.feed();
  }
}
