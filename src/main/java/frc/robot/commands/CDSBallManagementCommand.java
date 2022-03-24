// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CDSBallManagementCommand extends CommandBase {
  /** Creates a new CDSBallManagementCommand. */
  private final CDSSubsystem CDSSubsystem;

  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private String allianceColor;

  private int msCurrent = 0;
  private int ejectRuntime = 650; // amount of time auto eject will run intake backwards for in ms
  private int nextOpenSensor = -1; // placeholder
  private boolean[] sensorStatus;

  private ShuffleboardTab CDSTab = Shuffleboard.getTab("CDS Tab");
  private NetworkTableEntry autoEjectRunning = CDSTab.add("Auto Eject Running", false).getEntry();
  private NetworkTableEntry autoIntakeRunning = CDSTab.add("Auto Intake Running", false).getEntry();
  private NetworkTableEntry ballcountEntry = CDSTab.add("Ball Count", 0).getEntry();
  private NetworkTableEntry CDSState = CDSTab.add("CDS State", "IDLE").getEntry();

  public CDSBallManagementCommand(
      CDSSubsystem mCDSSubsystem,
      IntakeSubsystem mIntakeSubsystem,
      ShooterSubsystem mShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mCDSSubsystem);
    addRequirements(mIntakeSubsystem);
    addRequirements(mShooterSubsystem);

    CDSSubsystem = mCDSSubsystem;
    intakeSubsystem = mIntakeSubsystem;
    shooterSubsystem = mShooterSubsystem;

    allianceColor = CDSSubsystem.getAllianceColor();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int ballCount = CDSSubsystem.getBallCount();

    sensorStatus = CDSSubsystem.getSensorStatus();
    String state = CDSSubsystem.getState();

    switch (state) {
      case "IDLE":
        CDSSubsystem.stopCDS();
        intakeSubsystem.stopIntake();
        shooterSubsystem.runCargo(0.0);
        autoEjectRunning.setBoolean(false);
        autoIntakeRunning.setBoolean(false);

        msCurrent = 0;
        nextOpenSensor = -1;

        break;

      case "EJECT":
        intakeSubsystem.toggleIntake(true);
        CDSSubsystem.CDSWheelToggle(true);
        autoEjectRunning.setString("true");
        ballcountEntry.setNumber(ballCount);

        break;

      case "ADVANCE":
        CDSSubsystem.CDSWheelToggle(false);
        CDSSubsystem.CDSBeltToggle(false);
        shooterSubsystem.runCargo(Constants.reverseStopperWheelSpeed);
        autoIntakeRunning.setBoolean(true);
        ballcountEntry.setNumber(ballCount);

        break;
    }
    CDSState.setString(state.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
