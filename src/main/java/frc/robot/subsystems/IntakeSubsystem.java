// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry DIntakeSpeed =
      operatorTab
          .add("Intake Speed", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withPosition(3, 0)
          .withSize(2, 1)
          .getEntry();

  private MotorController intakeMotorControllerOne;
  private MotorController deployController;
  private SparkMaxPIDController deployPID;
  private RelativeEncoder deployEncoder;

  public IntakeSubsystem() {
    deployEncoder.setPosition(0);
    intakeMotorControllerOne = new MotorController("Intake Motor One", Constants.intakeMotorOneID, Constants.intakeDeployPID);
    deployController =
        new MotorController("Intake Deploy", Constants.intakeDeployMotorID);
    deployPID = deployController.getPIDCtrl();
    deployController.setIdleMode(IdleMode.kBrake);

    intakeMotorControllerOne.setInverted(true);
  }

  public void CDSDeployIntake(boolean deploy) {
    if (deploy) {
      deployPID.setReference(10, CANSparkMax.ControlType.kPosition);
    } else {
      deployPID.setReference(0, CANSparkMax.ControlType.kPosition);
    }
  }

  public void toggleIntake(boolean reverse) {
    // only runs intake if ball count isn't too high (addresses #140)
    if (reverse) {
      intakeMotorControllerOne.set(-Constants.intakeMotorSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("Intake Motor Direction", "Reverse");
        SmartDashboard.putNumber("Intake Motor Speed", -Constants.intakeMotorSpeed);
      }
      DIntakeSpeed.setDouble(-1);
    } else {
      intakeMotorControllerOne.set(Constants.intakeMotorSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("Intake Motor Direction", "Forward");
        SmartDashboard.putNumber("Intake Motor Speed", Constants.intakeMotorSpeed);
      }
      DIntakeSpeed.setDouble(1);
    }
  }

  public void stopIntake() {
    intakeMotorControllerOne.set(0.0);
    DIntakeSpeed.setDouble(0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("Intake Motor Speed", 0.0);
    }
  }

  public void periodic() {}
}
