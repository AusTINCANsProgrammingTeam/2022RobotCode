// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbUPCommand extends CommandBase {

  private final ClimbSubsystem m_subsystem;

  public ClimbUPCommand(ClimbSubsystem s) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s);
    m_subsystem = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.enableClimb(true, true);
    m_subsystem.periodic();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.enableClimb(false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}