// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeForwardCommand extends CommandBase {
  /** Creates a new IntakeForwardCommand. */
  private final IntakeSubsystem mIntakeSubsystem;

  private final CDSSubsystem mCdsSubsystem;

  public IntakeForwardCommand(IntakeSubsystem intakeSubsystem, CDSSubsystem cdsSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    addRequirements(cdsSubsystem);
    mIntakeSubsystem = intakeSubsystem;
    mCdsSubsystem = cdsSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntakeSubsystem.toggleIntake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntakeSubsystem.stopIntake();
    mCdsSubsystem.stopCDSWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
