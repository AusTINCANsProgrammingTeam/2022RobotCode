// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;


public class DriveBaseSubsystem extends SubsystemBase {

  private final Joystick m_driverJoystick;
  private final MotorController[] m_motorControllers;
  private final DifferentialDrive m_differentialDrive;
  private DifferentialDrivetrainSim m_DifferentialDrivetrainSim;
  public final Field2d m_field = new Field2d();
  private AnalogGyro m_gyro;
  private AnalogGyroSim m_gyroSim;
  public static ADIS16448_IMU m_gyro2; //Non-native gyro, might use later
  public static ADXRS450_Gyro m_gyro1;
  private final DifferentialDriveOdometry m_odometry;
  public static Encoder m_leftEncoder;
  public static Encoder m_rightEncoder;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  

  // Here are the encoders
  
  

  public DriveBaseSubsystem(Joystick joystick) { 
    m_leftEncoder = new Encoder(Constants.leftEncoderDIOone, Constants.leftEncoderDIOtwo, 
                                false, Encoder.EncodingType.k2X);

    m_rightEncoder = new Encoder(Constants.rightEncoderDIOone, Constants.rightEncoderDIOtwo, 
                                false, Encoder.EncodingType.k2X);

    m_driverJoystick = joystick;

    m_motorControllers = new MotorController[4];
    m_gyro1 = new ADXRS450_Gyro();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), new Pose2d(6.732000, 4.743371, new Rotation2d(2.70526)));
    
    

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

    if (Robot.isSimulation()) {
      m_gyro = new AnalogGyro(1);
      m_gyroSim = new AnalogGyroSim(m_gyro);

      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);

      SmartDashboard.putData("Field", m_field);
      
      m_DifferentialDrivetrainSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
        7.29,                    // 7.29:1 gearing reduction.
        10,                      // MOI of 7.5 kg m^2 (from CAD model).
        60.0,                    // mass of the robot
        Units.inchesToMeters(3), // The robot wheel radius
        1,                       // The track width
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
      );
        // standard deviations for measurement noise: x and y: 0.001m heading: 0.001 rad  l and r velocity: 0.1m/s  l and r position: 0.005m

    }
    // differential drive
    m_differentialDrive = new DifferentialDrive(m_motorControllers[Constants.driveLeftFrontIndex].getSparkMax(), 
                                          m_motorControllers[Constants.driveRightFrontIndex].getSparkMax());
  }

  @Override
  public void periodic() {

    // Update the smart dashboard in here, runs a for loop so it does it for every motor
    for(int i = 0; i < m_motorControllers.length; i++) {
      m_motorControllers[i].updateSmartDashboard();
    }
 
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
    if (Robot.isSimulation()) {
      m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), 
                                      -m_driverJoystick.getRawAxis(Constants.DBRightJoystickAxisX));
    } else {
      m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), 
                                      m_driverJoystick.getRawAxis(Constants.DBRightJoystickAxisY));
    }                                  
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  //TODO: Make a command to switch modes (only if we actually want this)
  public void arcadeDrive(double rotation) {
    m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.DBLeftJoystickAxisY), rotation);
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
    m_DifferentialDrivetrainSim.setInputs(-m_motorControllers[Constants.driveLeftFrontIndex].getSparkMax().get() * RobotController.getInputVoltage(),
                                          -m_motorControllers[Constants.driveRightFrontIndex].getSparkMax().get() * RobotController.getInputVoltage());

    m_gyroSim.setAngle(-m_DifferentialDrivetrainSim.getHeading().getDegrees());

    m_DifferentialDrivetrainSim.update(0.01);

    SmartDashboard.putNumber("LMotor",m_motorControllers[Constants.driveLeftFrontIndex].getSparkMax().get() );
    SmartDashboard.putNumber("RMotor",m_motorControllers[Constants.driveRightFrontIndex].getSparkMax().get() );

    m_leftEncoderSim.setDistance(m_DifferentialDrivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_DifferentialDrivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_DifferentialDrivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_DifferentialDrivetrainSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_DifferentialDrivetrainSim.getHeading().getDegrees());

    m_odometry.update(m_gyro.getRotation2d(),
                    m_leftEncoder.getDistance(),
                    m_rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());
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

  
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }
  

  // TODO: we can add more tankdrive co functions as extras later
}
