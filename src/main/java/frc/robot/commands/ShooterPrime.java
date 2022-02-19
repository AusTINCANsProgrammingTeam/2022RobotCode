// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPrime extends CommandBase {
  private ShooterSubsystem m_ShooterSubsystem;
  private LimelightSubsystem m_LimelightSubsystem;
  private CDSSubsystem m_CDSSubsystem;
  private int i;

  /** Creates a new ShooterPrimary. */
  public ShooterPrime(
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem,
      CDSSubsystem cdsSubsystem) {
    addRequirements(shooterSubsystem);
    m_ShooterSubsystem = shooterSubsystem;
    m_LimelightSubsystem = limelightSubsystem;
    // m_CDSSubsystem = cdsSubsystem;
    SmartDashboard.putBoolean("wheelReady", false);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.prime();
    if (m_ShooterSubsystem.wheelReady()) {
      if (i > 0 || m_LimelightSubsystem.calculatePID() == 0.0) {
        i++;
        m_ShooterSubsystem.runCargo(true, true);
        m_ShooterSubsystem.SetCargoBoolean(true);
        if (i >= 50) {// 1000 miliseconds delay
          i = 0;
          m_ShooterSubsystem.runCargo(false, false);
          m_ShooterSubsystem.SetCargoBoolean(false);
        }

      } else {
        m_ShooterSubsystem.runCargo(false, false);
      }
    }
      m_ShooterSubsystem.UpdateIdelay(i);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.runCargo(false, false);
    m_ShooterSubsystem.windFlywheel(0);
    m_ShooterSubsystem.SetCargoBoolean(false);
    m_ShooterSubsystem.UpdateIdelay(0);

    // SmartDashboard.putBoolean("wheelReady", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
