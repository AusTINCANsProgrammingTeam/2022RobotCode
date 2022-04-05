// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CDSBallManagementCommand extends CommandBase {
  /** Creates a new CDSBallManagementCommand. */
  private final CDSSubsystem CDSSubsystem;

  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterEject shooterEject;

  private int msBeltCurrent = 0;
  private int beltEjectRuntime = 100;

  private boolean firstRun = true;

  private static ShuffleboardTab CDSTab = Shuffleboard.getTab("CDS Tab");
  private static NetworkTableEntry autoEjectRunning =
      CDSTab.add("Auto Eject Running", false).getEntry();
  private static NetworkTableEntry autoIntakeRunning =
      CDSTab.add("Auto Intake Running", false).getEntry();

  public CDSBallManagementCommand(
      CDSSubsystem mCDSSubsystem,
      IntakeSubsystem mIntakeSubsystem,
      ShooterSubsystem mShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mCDSSubsystem);
    addRequirements(mIntakeSubsystem);
    addRequirements(mShooterSubsystem);

    CDSSubsystem = mCDSSubsystem;
    intakeSubsystem = mIntakeSubsystem;
    shooterSubsystem = mShooterSubsystem;
    shooterEject = new ShooterEject(shooterSubsystem, CDSSubsystem);
    shooterEject.initialize();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (CDSSubsystem.managementEnabled()) {
      CDSSubsystem.ManagementState state = CDSSubsystem.getState();

      switch (state) {
        case IDLE:
          CDSSubsystem.stopCDS();
          intakeSubsystem.stopIntake();
          shooterSubsystem.runCargo(0.0);
          autoEjectRunning.setBoolean(false);
          autoIntakeRunning.setBoolean(false);
          shooterEject.end(false);
          firstRun = true;

          break;

        case EJECT:
          if (msBeltCurrent <= beltEjectRuntime) {
            CDSSubsystem.CDSBeltToggle(true);
            msBeltCurrent += 20;
          } else {
            CDSSubsystem.stopCDSBelt();
          }

          intakeSubsystem.toggleIntake(true);
          CDSSubsystem.CDSWheelToggle(true);
          autoEjectRunning.setBoolean(true);

          break;

        case ADVANCE:
          intakeSubsystem
              .stopIntake(); // done in cases where transition from front eject to CDS is instant

          CDSSubsystem.CDSWheelToggle(false);
          CDSSubsystem.CDSBeltToggle(false);
          shooterSubsystem.runCargo(Constants.reverseStopperWheelSpeed);
          autoIntakeRunning.setBoolean(true);

          break;

        case SHOOTER_EJECT:
          CDSSubsystem.CDSWheelToggle(false);
          CDSSubsystem.CDSBeltToggle(false);
          autoEjectRunning.setBoolean(true);
          shooterSubsystem.runCargo(Constants.Shooter.cargoForward);

          if (firstRun) {
            // only run initialize once per state run
            shooterEject.initialize();
            firstRun = false;
          }
          shooterEject.execute();

          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
