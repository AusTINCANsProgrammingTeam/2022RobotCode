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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveBaseSubsystem extends SubsystemBase {

  private final Joystick m_driverJoystick;
  private final MotorController[] m_motorControllers = new MotorController[6];
  private final DifferentialDrive m_differentialDrive;
  private DifferentialDrivetrainSim m_DifferentialDrivetrainSim;
  public final Field2d m_field = new Field2d();
  private final double KvLinear = 1.98;
  private final double KaLinear = 0.2;
  private final double KvAngular = 1.5;
  private final double KaAngular = 0.3;
  private AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), new Pose2d(5.0, 13.5, new Rotation2d()));

  
  public DriveBaseSubsystem(Joystick joystick) {  
    m_driverJoystick = joystick;
    SmartDashboard.putData("Field", m_field);

    

    // motor controllers
    m_motorControllers[Constants.kDriveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.kDriveLeftFront);
    m_motorControllers[Constants.kDriveLeftMiddleIndex] = new MotorController("Differential Left Middle", Constants.kDriveLeftMiddle);
    m_motorControllers[Constants.kDriveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.kDriveLeftRear);
    m_motorControllers[Constants.kDriveRightFrontIndex] = new MotorController("Differential Right Front", Constants.kDriveRightFront);
    m_motorControllers[Constants.kDriveRightMiddleIndex] = new MotorController("Differential Right Middle", Constants.kDriveRightMiddle);
    m_motorControllers[Constants.kDriveRightRearIndex] = new MotorController("Differential Right Rear", Constants.kDriveRightRear);

    // inverses right side motors (2022 wpilib doesn't default it to be inverted for differential drive)
    m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax().setInverted(true);
    m_motorControllers[Constants.kDriveRightMiddleIndex].getSparkMax().setInverted(true);
    m_motorControllers[Constants.kDriveRightRearIndex].getSparkMax().setInverted(true);

    //Forces middle and rear motors of each side to follow the first
    m_motorControllers[Constants.kDriveLeftRearIndex].getSparkMax().follow(m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax());
    m_motorControllers[Constants.kDriveLeftMiddleIndex].getSparkMax().follow(m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax());
    m_motorControllers[Constants.kDriveRightRearIndex].getSparkMax().follow(m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
    m_motorControllers[Constants.kDriveRightMiddleIndex].getSparkMax().follow(m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax());

    m_differentialDrive = new DifferentialDrive(m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax(), m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax());

    if (Robot.isSimulation()) {
      m_DifferentialDrivetrainSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
        7.29,                    // 7.29:1 gearing reduction.
        7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
        60.0,                    // mass of the robot
        Units.inchesToMeters(3), // The robot wheel radius
        0.7112,                  // The track width
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
        // standard deviations for measurement noise: x and y: 0.001m heading: 0.001 rad  l and r velocity: 0.1m/s  l and r position: 0.005m
    }
  }

  @Override
  public void periodic() {
    // Update the smart dashboard in here, runs a for loop so it does it for every motor
    for(int i = 0; i < m_motorControllers.length; i++) {
      m_motorControllers[i].updateSmartDashboard();
      m_odometry.update(m_gyro.getRotation2d(),
                    m_motorControllers[Constants.kDriveLeftFrontIndex].getEncoder().getPosition(),
                    m_motorControllers[Constants.kDriveRightFrontIndex].getEncoder().getPosition());
      m_field.setRobotPose(m_odometry.getPoseMeters());

    }
 
  }

  // Normal Arcade Drive
  public void arcadeDrive() {
    m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), m_driverJoystick.getRawAxis(Constants.kDBRightJoystickAxisY));

  }

  // tank drive, not used but good to have
  public void tankDrive() {
    m_differentialDrive.tankDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), m_driverJoystick.getRawAxis(Constants.kDBRightJoystickAxisY));
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  public void arcadeDrive(double rotation) {
    m_differentialDrive.arcadeDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), rotation);
  }

  @Override
  public void simulationPeriodic() {
    m_DifferentialDrivetrainSim.setInputs(m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax().get() * RobotController.getInputVoltage(),
    m_motorControllers[Constants.kDriveRightFrontIndex].getSparkMax().get() * RobotController.getInputVoltage());
    m_gyroSim.setAngle(-m_DifferentialDrivetrainSim.getHeading().getDegrees());
    m_DifferentialDrivetrainSim.update(0.02);
  }

  public void driveFunction() {
    // currently serves no purpose
  }

  public void stopMotorsFunction() {
    // Calls Arcade Drive with a zero to both speed and rotation in order to stop the motors
    m_differentialDrive.arcadeDrive(0.0, 0.0);
  }

  // TODO: return actual speeds
  public double getLeftSpeed() {
    return 0.0;
  }

  public double getRightSpeed() {
    return 0.0;
  }

  // TODO: we can add more tankDrive co functions as extras later
}
