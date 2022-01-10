// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveBaseTeleopCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveBaseSubsystem m_subsystem; //Put subsystem here

  public DriveBaseTeleopCommand(DriveBaseSubsystem s) {
    addRequirements(s);
    m_subsystem = s;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //m_subsystem.driveFunction();
  }
  @Override
  public void end(boolean interrupted) {
    //m_subsystem.stopMotorsFunction();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
