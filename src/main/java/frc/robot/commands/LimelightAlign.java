// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightAlign extends CommandBase {
  private LimelightSubsystem m_LimelightSubsystem;
  private DriveBaseSubsystem m_drivebaseSubsystem;

  /** Creates a new ShooterPrimary. */
  public LimelightAlign(
      LimelightSubsystem limelightSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
    addRequirements(limelightSubsystem);
    addRequirements(driveBaseSubsystem);
    m_LimelightSubsystem = limelightSubsystem;
    m_drivebaseSubsystem = driveBaseSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double adjustment = m_LimelightSubsystem.calculatePID();
    m_drivebaseSubsystem.setSpeeds(adjustment*-1,adjustment );
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
