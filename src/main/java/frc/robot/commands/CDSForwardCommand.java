// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.StopperSubsystem;

public class CDSForwardCommand extends CommandBase {
  private CDSSubsystem CDSSubsystem;
  private StopperSubsystem stopperSubsystem;

  public CDSForwardCommand(CDSSubsystem CDSSubsystem, StopperSubsystem stopperSubsystem) {
    addRequirements(CDSSubsystem);
    addRequirements(stopperSubsystem);
    this.CDSSubsystem = CDSSubsystem;
    this.stopperSubsystem = stopperSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CDSSubsystem.runCDS(false);
    stopperSubsystem.reverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CDSSubsystem.stopCDS();
    stopperSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
