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
// import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import frc.robot.common.hardware.MotorController.MotorConfig;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean intakeDeployed;
  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake Tab");

  private NetworkTableEntry DIntakeSpeed =
      operatorTab
          .add("Intake Speed", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withPosition(1, 3)
          .withSize(2, 1)
          .getEntry();

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;

  private SparkMaxPIDController deployPIDController;
  private RelativeEncoder deployEncoder;

  public IntakeSubsystem() {
    intakeDeployed = false;
    intakeMotor = MotorController.constructMotor(MotorConfig.intakeMotor);
    deployMotor = MotorController.constructMotor(MotorConfig.intakeDeploy);
    deployPIDController = MotorController.constructPIDController(deployMotor, Constants.intakeDeployPID);
    deployEncoder = deployMotor.getEncoder();
    deployMotor.setIdleMode(IdleMode.kBrake);
    deployEncoder.setPosition(0);
  }

  public boolean getIntakeDeployed() {
    return intakeDeployed;
  }

  public void toggleIntake(boolean reverse) {
    // only runs intake if ball count isn't too high (addresses #140)
    if (reverse) {
      intakeMotor.set(-Constants.intakeMotorSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("Intake Motor Direction", "Reverse");
        SmartDashboard.putNumber("Intake Motor Speed", -Constants.intakeMotorSpeed);
      }
      DIntakeSpeed.setDouble(-1);
    } else {
      intakeMotor.set(Constants.intakeMotorSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("Intake Motor Direction", "Forward");
        SmartDashboard.putNumber("Intake Motor Speed", Constants.intakeMotorSpeed);
      }
      DIntakeSpeed.setDouble(1);
    }
  }

  public void deployIntake() {
    deployPIDController.setReference(Constants.intakeDeployPos, CANSparkMax.ControlType.kPosition);
    intakeDeployed = true;
  }

  public void retractIntake() {
    deployPIDController.setReference(Constants.intakeRetractPos, CANSparkMax.ControlType.kPosition);
    intakeDeployed = false;
  }

  public void stopIntake() {
    intakeMotor.set(0.0);
    DIntakeSpeed.setDouble(0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("Intake Motor Speed", 0.0);
    }
  }

  public void periodic() {
    SmartDashboard.putBoolean("Intake Out?", intakeDeployed);
    SmartDashboard.putNumber("Deploy Encoder", deployMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Applied Output, deploy", deployMotor.getAppliedOutput());
  }
}
