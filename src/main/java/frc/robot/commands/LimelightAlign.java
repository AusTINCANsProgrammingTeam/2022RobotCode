// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightAlign extends CommandBase {
  private LimelightSubsystem m_LimelightSubsystem;

  /** Creates a new ShooterPrimary. */
  public LimelightAlign(LimelightSubsystem limelightSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
    if(limelightSubsystem != null) {
      addRequirements(limelightSubsystem);
    }
    if(driveBaseSubsystem != null) {
      addRequirements(driveBaseSubsystem);
    }
    m_LimelightSubsystem = limelightSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LimelightSubsystem.setMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LimelightSubsystem.stopMotors();
    m_LimelightSubsystem.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_LimelightSubsystem.getFinished();
  }
}
