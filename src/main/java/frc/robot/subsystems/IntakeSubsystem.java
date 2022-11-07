// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.common.hardware.MotorController;
import frc.robot.common.hardware.MotorController.MotorConfig;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;

  private SparkMaxPIDController deployPIDController;
  private RelativeEncoder deployEncoder;
  
  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry intakeIndicator =
      operatorTab
          .add("Intake Indicator", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withPosition(1, 3)
          .withSize(2, 1)
          .getEntry();
  private NetworkTableEntry deployIndicator =
      operatorTab
          .add("Deploy Indicator", false)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .withPosition(3, 2)
          .getEntry();

  public IntakeSubsystem() {
    intakeMotor = MotorController.constructMotor(MotorConfig.intakeMotor);
    deployMotor = MotorController.constructMotor(MotorConfig.intakeDeploy);
    deployMotor.setIdleMode(IdleMode.kBrake);

    deployPIDController = MotorController.constructPIDController(deployMotor, IntakeConstants.PIDArray);

    deployEncoder = deployMotor.getEncoder();
    deployEncoder.setPosition(0);
  }

  public void runIntake(boolean reversed) {
    if (reversed) {
      intakeIndicator.setDouble(-1);
      intakeMotor.set(-IntakeConstants.intakeSpeed);
    } else {
      intakeIndicator.setDouble(-1);
      intakeMotor.set(IntakeConstants.intakeSpeed);
    }
  }

  public void deployIntake() {
    deployIndicator.setBoolean(true);
    deployPIDController.setReference(IntakeConstants.deployPosition, CANSparkMax.ControlType.kPosition);
  }

  public void retractIntake() {
    deployIndicator.setBoolean(false);
    deployPIDController.setReference(IntakeConstants.retractPosition, CANSparkMax.ControlType.kPosition);
  }

  public void stopIntake() {
    intakeIndicator.setDouble(0);
    intakeMotor.set(0.0);
  }

  public void periodic() {}
}
