// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightAlign extends CommandBase {
  private LimelightSubsystem m_LimelightSubsystem;
  private DriveBaseSubsystem m_drivebaseSubsystem;

  /** Creates a new ShooterPrimary. */
  public LimelightAlign(LimelightSubsystem limelightSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
    SmartDashboard.putNumber("llrun", 0);
    addRequirements(limelightSubsystem);
    addRequirements(driveBaseSubsystem);
    m_LimelightSubsystem = limelightSubsystem;
    m_drivebaseSubsystem = driveBaseSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("llrun", 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("exec", 1);
    double adjustment = m_LimelightSubsystem.calculatePID();
    SmartDashboard.putNumber("adjust", adjustment);
    m_drivebaseSubsystem.setSpeeds(adjustment,adjustment*-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebaseSubsystem.setSpeeds(0,0);
    m_LimelightSubsystem.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_LimelightSubsystem.getFinished();
  }
}
