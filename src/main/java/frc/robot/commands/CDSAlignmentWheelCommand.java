// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CDSSubsystem;

public class CDSAlignmentWheelCommand extends CommandBase {
  /** Creates a new CDSAlignmentWheelCommand. */
  private final CDSSubsystem mCDSAlignmentWheel;

  public CDSAlignmentWheelCommand(CDSSubsystem CDSSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CDSSubsystem);
    mCDSAlignmentWheel = CDSSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mCDSAlignmentWheel.CDSAlignmentToggle(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mCDSAlignmentWheel.stopCDS();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
