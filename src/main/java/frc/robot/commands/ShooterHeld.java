// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterHeld extends CommandBase {
  private ShooterSubsystem m_ShooterSubsystem;
  private LimelightSubsystem m_LimelightSubsystem;
  private CDSSubsystem m_CDSSubsystem;
  private int i;
  private boolean LLEnabled;

  /** Creates a new ShooterHeld. */
  public ShooterHeld(
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem,
      CDSSubsystem CDSSubsystem,
      boolean llEnabled) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CDSSubsystem);
    addRequirements(shooterSubsystem);
    if (llEnabled) {
      addRequirements(limelightSubsystem);
    }
    m_ShooterSubsystem = shooterSubsystem;
    m_LimelightSubsystem = limelightSubsystem;
    m_CDSSubsystem = CDSSubsystem;
    SmartDashboard.putBoolean("wheelReady", false);
    LLEnabled = llEnabled;
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
      // If below will bypass the LL check if the stopper is already running, or the LL is disabled.
      // Otherwise, alignment is checked.
      if (i > 0 || !LLEnabled || m_LimelightSubsystem.calculatePID() == 0.0) {
        if (i == 0) {
          m_CDSSubsystem.CDSBeltToggle(false);
          m_ShooterSubsystem.runCargo(Constants.Shooter.cargoForward);
          m_ShooterSubsystem.setCargoBoolean(true);
        }
        i++;
        if (i >= 50) { // 1000 miliseconds delay TODO: Use a CDS method for this when possible
          i = 0;
          m_CDSSubsystem.stopCDS();
          m_ShooterSubsystem.runCargo(0);
          m_ShooterSubsystem.setCargoBoolean(false);
        }
      }
    } else {
      m_ShooterSubsystem.setCargoBoolean(false);
      m_CDSSubsystem.stopCDS();
      m_ShooterSubsystem.runCargo(Constants.Shooter.cargoReverse);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.runCargo(0);
    m_ShooterSubsystem.windFlywheel(0);
    m_ShooterSubsystem.setCargoBoolean(false);
    m_CDSSubsystem.stopCDS();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
