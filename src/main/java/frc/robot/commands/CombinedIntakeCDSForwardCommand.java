// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.CDSSubsystem.ManagementState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CombinedIntakeCDSForwardCommand extends CommandBase {
  /** Creates a new OuttakeCommand. */
  private final CDSSubsystem CDSSubsystem;

  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private ManagementState lastState;

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
    lastState = ManagementState.EJECT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CDSSubsystem.CDSBeltToggle(false, Constants.CDSBeltSpeed);
    CDSSubsystem.CDSWheelToggle(false);
    intakeSubsystem.toggleIntake(false);
    shooterSubsystem.runCargo(Constants.Shooter.cargoReverse);
    intakeSubsystem.deployIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CDSSubsystem.stopCDS();
    intakeSubsystem.stopIntake();
    shooterSubsystem.runCargo(0.0);
    intakeSubsystem.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
