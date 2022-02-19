// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OuttakeCommand extends CommandBase {
  /** Creates a new OuttakeCommand. */
  private final CDSSubsystem CDSSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public OuttakeCommand(IntakeSubsystem mIntakeSubsystem, CDSSubsystem mCDSSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeSubsystem);
    addRequirements(mCDSSubsystem);
    intakeSubsystem = mIntakeSubsystem;
    CDSSubsystem = mCDSSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CDSSubsystem.CDSBeltToggle(true);
    CDSSubsystem.CDSWheelToggle(true);
    intakeSubsystem.toggleIntake(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CDSSubsystem.stopCDS();
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
