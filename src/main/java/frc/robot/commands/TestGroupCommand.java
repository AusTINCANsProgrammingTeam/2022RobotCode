// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestGroupCommand extends SequentialCommandGroup {
  public TestGroupCommand(DriveBaseSubsystem drive, IntakeSubsystem intake){
    addCommands(
      new IntakeForwardCommand(intake),
      new AutonCommand(drive)
      

    );
  }
private void addCommands(IntakeForwardCommand intakeForwardCommand, AutonCommand autonCommand) {
}

  

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  
}
