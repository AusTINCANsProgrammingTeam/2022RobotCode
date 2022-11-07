// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StopperSubsystem;

public class ShooterPressed extends CommandBase {
  private ShooterSubsystem m_ShooterSubsystem;
  private CDSSubsystem m_CDSSubsystem;
  private StopperSubsystem stopperSubsystem;
  private int i;

  /** Creates a new ShooterPressed. */
  public ShooterPressed(
      ShooterSubsystem shooterSubsystem,
      CDSSubsystem cdsSubsystem,
      StopperSubsystem stopperSubsystem) {
    addRequirements(shooterSubsystem);
    addRequirements(stopperSubsystem);
    addRequirements(cdsSubsystem);
    m_ShooterSubsystem = shooterSubsystem;
    m_CDSSubsystem = cdsSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    m_ShooterSubsystem.resetIAccum();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.windShooter();
    if (m_ShooterSubsystem.wheelReady() || i > 0) {
      // If below will bypass the LL check if the stopper is already running, or the LL is disabled.
      // Otherwise, alignment is checked.
      if (i > 0) {
        // if (i == 0) {
        m_CDSSubsystem.runBelt(false);
        stopperSubsystem.forward();
        // }
        i++;
      }
    } else if (i == 0) {
      // when wheel is not ready and i is still 0
      m_CDSSubsystem.stopCDS();
      stopperSubsystem.forward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CDSSubsystem.stopCDS();
    stopperSubsystem.stop();
    m_ShooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: use CDSSubsystem getBallCount to check if all balls are shot yet

    if (i >= 26) { // 20 * 20 = 400 miliseconds timeout
      return true;
    }
    return false;
  }
}
