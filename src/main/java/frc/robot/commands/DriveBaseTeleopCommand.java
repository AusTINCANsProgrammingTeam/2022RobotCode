// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBaseSubsystem;

public class DriveBaseTeleopCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveBaseSubsystem subsystem;

  public DriveBaseTeleopCommand(DriveBaseSubsystem s) {
    addRequirements(s);
    subsystem = s;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    subsystem.tankDrive();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stopDriveMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
