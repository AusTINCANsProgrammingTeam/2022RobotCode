// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CDSSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.IntakeReverseCommand;

public class CDSAutoEjectCommand extends CommandBase {
  /** Creates a new CDSAutoEjectCommand. */
  IntakeSubsystem intakeSubsystem;
  CDSSubsystem CDSSubsystem;
  OuttakeCommand outtakeCommand;
  IntakeReverseCommand intakeReverseCommand;
  boolean ejectRunning;

  int msDelay = 2000;
  int cycleCount = 0;

  public CDSAutoEjectCommand(CDSSubsystem mCdsSubsystem, IntakeSubsystem mIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mCdsSubsystem);
    addRequirements(mIntakeSubsystem);
    intakeSubsystem = mIntakeSubsystem;
    CDSSubsystem = mCdsSubsystem;

    outtakeCommand = new OuttakeCommand(intakeSubsystem, CDSSubsystem);
    intakeReverseCommand = new IntakeReverseCommand(intakeSubsystem, CDSSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int ballCount = (int) SmartDashboard.getNumber("Ball Count", 0);
    
    if (!ejectRunning) {
      if (ballCount > 2) {
        intakeReverseCommand.initialize();
        ejectRunning = true;
      }
    } else {
      if (cycleCount * 20 >= msDelay) {
        intakeReverseCommand.end(true);
      } else {
        cycleCount++;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeReverseCommand.end(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
