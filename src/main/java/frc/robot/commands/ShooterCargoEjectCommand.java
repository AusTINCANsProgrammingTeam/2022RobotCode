// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ShooterCargoEjectCommand extends CommandBase {
  /** Creates a new ShooterCargoEjectCommand. */
  private ShooterSubsystem shooterSubsystem;
  private CDSSubsystem CDSSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private Boolean llEnabled;
  private int i;

  public ShooterCargoEjectCommand(ShooterSubsystem mShooterSubsystem, CDSSubsystem mCDSSubsystem, LimelightSubsystem mLimelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooterSubsystem);
    addRequirements(mCDSSubsystem);
    if (llEnabled){
      addRequirements(mLimelightSubsystem);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
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
    return false;
  }
}
