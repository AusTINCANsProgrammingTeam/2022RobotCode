// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterPrime extends CommandBase {
  private ShooterSubsystem m_ShooterSubsystem;
  private LimelightSubsystem m_LimelightSubsystem;
  private int i;

  /** Creates a new ShooterPrimary. */
  public ShooterPrime(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    addRequirements(shooterSubsystem);
    m_ShooterSubsystem = shooterSubsystem;
    m_LimelightSubsystem = limelightSubsystem;
    SmartDashboard.putBoolean("wheelReady", false);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.prime();
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.windFlywheel(5.0);;
    i = 0;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.runCargo(false);
    m_ShooterSubsystem.windFlywheel(0);
    SmartDashboard.putBoolean("wheelReady", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_ShooterSubsystem.wheelReady()){
      SmartDashboard.putBoolean("wheelReady", true);
      if(i > 0 || m_LimelightSubsystem.calculatePID() == 0.0){
        m_ShooterSubsystem.runCargo(true);
        i++;
        if(i==10){ //Expected to add a 200ms delay
          return true;
        }
      }
    }
    return false;
  }
}
