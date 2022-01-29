// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class DriveBaseSim extends SubsystemBase {
  private final Joystick m_driverJoystick;
  private final MotorController[] m_motorControllers = new MotorController[6];
  static final double KvLinear = 1.98;
  final double KaLinear = 0.2;
  static final double KvAngular = 1.5;
  static final double KaAngular = 0.3;
  // Create the simulation model of our drivetrain.
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), 7.29, 7.5, 60.0, Units.inchesToMeters(3), 0.7112, VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  private final DifferentialDrivetrainSim m_DifferentialDrivetrainSim;
  /** Creates a new DriveBaseSim. */
  public DriveBaseSim(Joystick joystick) {
    m_driverJoystick = joystick;

    m_DifferentialDrivetrainSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
      7.29,                    // 7.29:1 gearing reduction.
      7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
      60.0,                    // The mass of the robot is 60 kg.
      Units.inchesToMeters(3), // The robot uses 3" radius wheels.
      0.7112,                  // The track width is 0.7112 meters.
      // The standard deviations for measurement noise: x and y: 0.001m heading: 0.001 rad  l and r velocity: 0.1m/s  l and r position: 0.005m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));  }

        // Normal Arcade Drive
  public void arcadeDrive() {
    //m_DifferentialDrivetrainSim.arcadeDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), m_driverJoystick.getRawAxis(Constants.kDBRightJoystickAxisY));

  }

  // tank drive, not used but good to have
  public void tankDrive() {
    //m_DifferentialDrivetrainSim.tankDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), m_driverJoystick.getRawAxis(Constants.kDBRightJoystickAxisY));
  }

  // Arcade Drive where you can only move forwards and backwards for testing
  //public void arcadeDrive(double rotation) {
  m_DifferentialDrivetrainSim.arcadeDrive(m_driverJoystick.getRawAxis(Constants.kDBLeftJoystickAxisY), rotation);
  //}

  @Override
  public void simulationPeriodic() {
    // Currently serves no purpose
  }

  public void driveFunction() {
    // currently serves no purpose
  }

  public void stopMotorsFunction() {
    // Calls Arcade Drive with a zero to both speed and rotation in order to stop the motors
    m_DifferentialDrivetrainSim.arcadeDrive(0.0, 0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    }
  }
