// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbSequence1 extends CommandBase {
  private final ClimbSubsystem climbSubsystem;

  /** Creates a new ClimbKeepDown. */
  public ClimbSequence1(ClimbSubsystem s) {
    addRequirements(s);

    this.climbSubsystem = s;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.setAutoBoolean(true);
    climbSubsystem.deployPoles();
    climbSubsystem.unlockHooks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.deployArms();
    climbSubsystem.resetClimbHeights();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.setAutoBoolean(false);
    climbSubsystem.resetClimbHeights();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbSubsystem.atFirstSetpoint();
  }
}
