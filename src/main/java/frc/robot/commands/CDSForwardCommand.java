// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.ShooterSubsystem; 

public class CDSForwardCommand extends CommandBase {
  /** Creates a new IntakeForwardCommand. */
  private final CDSSubsystem mCDSSubsystem;
  private final ShooterSubsystem mShooterSubsystem;
  
  public CDSForwardCommand(CDSSubsystem CDSSubsystem, ShooterSubsystem shooterSubsystem) {
    //Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CDSSubsystem);
    addRequirements(shooterSubsystem);
    mShooterSubsystem = shooterSubsystem;
    mCDSSubsystem = CDSSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mCDSSubsystem.CDSBeltWheelControllerToggle(false);
    mShooterSubsystem.runCargo(true, false);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mCDSSubsystem.stopCDS();
    mShooterSubsystem.runCargo(false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
