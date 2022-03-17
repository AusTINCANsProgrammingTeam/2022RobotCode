// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CombinedIntakeCDSForwardCommand extends CommandBase {
  /** Creates a new OuttakeCommand. */
  private final CDSSubsystem CDSSubsystem;

  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public CombinedIntakeCDSForwardCommand(
      IntakeSubsystem mIntakeSubsystem,
      CDSSubsystem mCDSSubsystem,
      ShooterSubsystem mShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooterSubsystem);
    addRequirements(mIntakeSubsystem);
    addRequirements(mCDSSubsystem);
    addRequirements(mShooterSubsystem);
    intakeSubsystem = mIntakeSubsystem;
    CDSSubsystem = mCDSSubsystem;
    shooterSubsystem = mShooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CDSSubsystem.CDSBeltToggle(false);
    CDSSubsystem.CDSWheelToggle(false);
    intakeSubsystem.toggleIntake(false);
    shooterSubsystem.runCargo(Constants.reverseStopperWheelSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CDSSubsystem.stopCDS();
    intakeSubsystem.stopIntake();
    shooterSubsystem.runCargo(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
