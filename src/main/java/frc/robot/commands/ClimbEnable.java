// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;

public class ClimbEnable extends CommandBase {
  private final ClimbSubsystem m_subsystem;
  private final DriveBaseSubsystem m_drivesubsystem;

  /** Creates a new ClimbEnable. */
  public ClimbEnable(ClimbSubsystem s, DriveBaseSubsystem d) {
    addRequirements(s);
    addRequirements(d);
    m_subsystem = s;
    m_drivesubsystem = d;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.climbEnable();
    // m_subsystem.resetTargetedHeight();

    if (m_subsystem.getclimbingmode()) {
      m_drivesubsystem.setArcadedrivespeed(40);
    } else {
      m_drivesubsystem.setArcadedrivespeed(100);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
