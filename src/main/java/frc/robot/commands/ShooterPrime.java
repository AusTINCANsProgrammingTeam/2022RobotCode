// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterPrime extends CommandBase {
  private ShooterSubsystem m_ShooterSubsystem;

  /** Creates a new ShooterPrimary. */
  public ShooterPrime(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    m_ShooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.windFlywheelTest(3600);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   //CHANGED FOR TESTING
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.windFlywheelTest(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 //   if(m_ShooterSubsystem.wheelReady()){
 //     m_ShooterSubsystem.shoot();
 //     return true;
 //   }
    return false;
  }
}
