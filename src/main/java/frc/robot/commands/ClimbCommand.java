// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;

public class ClimbCommand extends CommandBase {

  private final ClimbSubsystem m_subsystem;
  private final DriveBaseSubsystem m_drivesubsystem;

  public ClimbCommand(ClimbSubsystem s,DriveBaseSubsystem  d) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s);
    addRequirements(d);
    m_subsystem = s;
    m_drivesubsystem = d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_subsystem.getclimbingmode()) {
      m_drivesubsystem.setArcadedrivespeed(30);
      m_subsystem.resetTargetedHeight();
      m_subsystem.runManual();
    } else {
      m_subsystem.enableClimb();
    }
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
