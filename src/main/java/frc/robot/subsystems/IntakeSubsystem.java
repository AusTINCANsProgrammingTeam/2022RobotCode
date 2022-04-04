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

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean intakeDeployed;
  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake Tab");
  private NetworkTableEntry sbintakeDeployP =
      intakeTab.add("Intake deploy P", 0).withSize(1, 1).withPosition(0, 1).getEntry();
  private NetworkTableEntry sbintakeDeployI =
      intakeTab.add("Intake deploy I", 0).withSize(1, 1).withPosition(0, 2).getEntry();
  private NetworkTableEntry sbintakeDeployD =
      intakeTab.add("Intake deploy D", 0).withSize(1, 1).withPosition(0, 3).getEntry();
  private NetworkTableEntry sbintakeDeployMaxIAcum =
      intakeTab.add("Intake deploy I Max Acum", 0).withSize(1, 1).withPosition(0, 4).getEntry();
  private NetworkTableEntry sbintakeDeployCurrenttLimit =
      intakeTab.add("Intake deploy current limit", 0).withSize(1, 1).withPosition(0, 5).getEntry();
  private NetworkTableEntry sbintakeDeployPosition =
      intakeTab.add("Intake deploy position", 0).withSize(1, 1).withPosition(1, 1).getEntry();
  private NetworkTableEntry sbintakeDeployed =
      intakeTab.add("Intake Deployed", false).withSize(1, 1).withPosition(1, 2).getEntry();

  private NetworkTableEntry DIntakeSpeed =
      operatorTab
          .add("Intake Speed", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withPosition(1, 3)
          .withSize(2, 1)
          .getEntry();

  private MotorController intakeMotorControllerOne;
  private MotorController deployController;
  private SparkMaxPIDController deployPID;
  private RelativeEncoder deployEncoder;

  public IntakeSubsystem() {
    intakeDeployed = false;
    intakeMotorControllerOne =
        new MotorController(
            "Intake Motor One", Constants.intakeMotorOneID, Constants.intakeDeployPID);
    deployController = new MotorController("Intake Deploy", Constants.intakeDeployMotorID);
    deployPID = deployController.getPIDCtrl();
    deployEncoder = deployController.getEncoder();
    deployController.setIdleMode(IdleMode.kBrake);
    deployEncoder.setPosition(0);

    intakeMotorControllerOne.setInverted(true);
  }

  public void resetpid() {
    deployPID.setP(sbintakeDeployP.getDouble(0));
    deployPID.setI(sbintakeDeployI.getDouble(0));
    deployPID.setD(sbintakeDeployD.getDouble(0));
    deployPID.setIMaxAccum(sbintakeDeployMaxIAcum.getDouble(0), 0);
  }




  public boolean getIntakeDeployed() {
    return intakeDeployed;
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

  public void deployIntake() {
    intakeDeployed = !intakeDeployed;
    if (intakeDeployed) {
      deployPID.setReference(10, CANSparkMax.ControlType.kPosition);
    } else {
      deployPID.setReference(0, CANSparkMax.ControlType.kPosition);
    }
  }

  public void stopIntake() {
    intakeMotorControllerOne.set(0.0);
    DIntakeSpeed.setDouble(0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("Intake Motor Speed", 0.0);
    }
  }

  public void periodic() {
    if ((sbintakeDeployP.getDouble(0) != deployPID.getP())
        | (sbintakeDeployI.getDouble(0) != deployPID.getI())
        | (sbintakeDeployD.getDouble(0) != deployPID.getD())
        | (sbintakeDeployMaxIAcum.getDouble(0) != deployPID.getIMaxAccum(0))) {
      resetpid();
    }
    sbintakeDeployed.setBoolean(intakeDeployed);
    sbintakeDeployPosition.setDouble(deployEncoder.getPosition());

    deployController.updateSmartDashboard();
    SmartDashboard.putBoolean("Intake Out?", intakeDeployed);
    SmartDashboard.putNumber("Deploy Encoder", deployEncoder.getPosition());
  }
}
