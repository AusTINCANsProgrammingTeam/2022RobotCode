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
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;


public class ElevatorSim extends SubsystemBase {
  private int m_elevatorGearbox = 1;
  private int kElevatorGearing = 1;
  private int kCarriageMass = 10;
  private int kElevatorDrumRadius = 10;
  private int kMinElevatorHeight = 1;
  private int kMaxElevatorHeight = 10;
  private final ElevatorSim m_elevatorSim =
  new ElevatorSim(
      m_elevatorGearbox,
      kElevatorGearing,
      kCarriageMass,
      kElevatorDrumRadius,
      kMinElevatorHeight,
      kMaxElevatorHeight,
      VecBuilder.fill(0.01));
      private final EncoderSim m_encoderSim = new EncoderSim(m_encoderSim);
  /** Creates a new ElevatorSim. */
  public ElevatorSim() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
