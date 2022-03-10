// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AimModes;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterEject extends CommandBase {
  private ShooterSubsystem m_ShooterSubsystem;
  private CDSSubsystem m_CDSSubsystem;
  private int i;
  private AimModes tempMode;

  /** Creates a new ShooterPressed. */
  public ShooterEject(ShooterSubsystem shooterSubsystem, CDSSubsystem CDSSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    addRequirements(CDSSubsystem);
    m_ShooterSubsystem = shooterSubsystem;
    m_CDSSubsystem = CDSSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tempMode = m_ShooterSubsystem.getAimMode();
    i = 0;
    m_ShooterSubsystem.setAimMode(Constants.AimModes.EJECT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.prime();
    if (m_ShooterSubsystem.wheelReady()) {
      if (i == 0) {
        m_CDSSubsystem.CDSBeltToggle(false);
        m_ShooterSubsystem.runCargo(Constants.Shooter.cargoForward);
        m_ShooterSubsystem.setCargoBoolean(true);
      }
      i++;
    } else {
      m_CDSSubsystem.stopCDS();
      m_ShooterSubsystem.runCargo(Constants.Shooter.cargoReverse);
      m_ShooterSubsystem.setCargoBoolean(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setAimMode(tempMode);
    m_ShooterSubsystem.runCargo(0);
    m_ShooterSubsystem.windFlywheel(0);
    m_ShooterSubsystem.setCargoBoolean(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (i >= 50) { // 1000 milliseconds delay TODO: CDS method is critical for this!!!
      return true;
    }
    return false;
  }
}
