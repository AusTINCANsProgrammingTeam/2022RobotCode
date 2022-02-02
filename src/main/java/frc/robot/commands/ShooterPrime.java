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
    m_ShooterSubsystem.prime();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted){
      //TODO: uncomment when cargoMotor exists 
      m_ShooterSubsystem.shoot();
    }
    m_ShooterSubsystem.windFlywheel(0); //TODO: Look at this again, we don't want our flywheel to instantly switch off in the end
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_ShooterSubsystem.wheelReady()){
      return true;
    }
    return false;
  }
}
