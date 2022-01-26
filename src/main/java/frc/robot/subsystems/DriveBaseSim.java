// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class DriveBaseSim extends SubsystemBase {
  private final Joystick m_driverJoystick;
  private final MotorController[] m_motorControllers = new MotorController[6];
  private final DifferentialDrivetrainSim m_dDifferentialDrivetrainSim;
  /** Creates a new DriveBaseSim. */
  public DriveBaseSim(Joystick joystick) {
    m_driverJoystick = joystick;

    // motor controllers
    m_motorControllers[Constants.kDriveLeftFrontIndex] = new MotorController("DifferentialSim Left Front", Constants.kDriveSimLeftFront);
    m_motorControllers[Constants.kDriveLeftMiddleIndex] = new MotorController("DifferentialSim Left Middle", Constants.kDriveSimLeftMiddle);
    m_motorControllers[Constants.kDriveLeftRearIndex] = new MotorController("DifferentialSim Left Rear", Constants.kDriveSimLeftRear);
    m_motorControllers[Constants.kDriveRightFrontIndex] = new MotorController("DifferentialSim Right Front", Constants.kDriveSimRightFront);
    m_motorControllers[Constants.kDriveRightMiddleIndex] = new MotorController("DifferentialSim Right Middle", Constants.kDriveSimRightMiddle);
    m_motorControllers[Constants.kDriveRightRearIndex] = new MotorController("DifferentialSim Right Rear", Constants.kDriveSimRightRear);

    // inverses right side motors
    m_motorControllers[Constants.kDriveSimRightFrontIndex].getSparkMax().setInverted(true);
    m_motorControllers[Constants.kDriveSimRightMiddleIndex].getSparkMax().setInverted(true);
    m_motorControllers[Constants.kDriveSimRightRearIndex].getSparkMax().setInverted(true);

    // middle and rear motors follow front
    m_motorControllers[Constants.kDriveSimLeftRearIndex].getSparkMax().follow(m_motorControllers[Constants.kDriveSimLeftFrontIndex].getSparkMax());
    m_motorControllers[Constants.kDriveSimLeftMiddleIndex].getSparkMax().follow(m_motorControllers[Constants.kDriveSimLeftFrontIndex].getSparkMax());
    m_motorControllers[Constants.kDriveSimRightRearIndex].getSparkMax().follow(m_motorControllers[Constants.kDriveSimRightFrontIndex].getSparkMax());
    m_motorControllers[Constants.kDriveSimRightMiddleIndex].getSparkMax().follow(m_motorControllers[Constants.kDriveSimRightFrontIndex].getSparkMax());

    m_dDifferentialDrivetrainSim = new DifferentialDrivetrainSim(m_motorControllers[Constants.kDriveSimLeftFrontIndex].getSparkMax(), m_motorControllers[Constants.kDriveSimRightFrontIndex].getSparkMax());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
