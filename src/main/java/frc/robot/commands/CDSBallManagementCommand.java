// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CDSBallManagementCommand extends CommandBase {
  
  // TODO: Add a ADVANCE state 
  public enum ManagementState {
    IDLE,
    EJECT,
  }

  /** Creates a new CDSBallManagementCommand. */
  private final CDSSubsystem CDSSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private ManagementState state = ManagementState.IDLE;
  private String allianceColor;

  private int msCurrent = 0;
  private int ejectRuntime = 750; // amount of time auto eject will run intake backwards for in ms
  
  private ShuffleboardTab CDSTab = Shuffleboard.getTab("CDS Tab");
  private NetworkTableEntry autoEjectRunning = 
      CDSTab.add("Auto Eject Running", false).getEntry();
  private NetworkTableEntry autoIntakeRunning = 
      CDSTab.add("Auto Intake Running", false).getEntry();

  public CDSBallManagementCommand(CDSSubsystem mCDSSubsystem, IntakeSubsystem mIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mCDSSubsystem);
    addRequirements(mIntakeSubsystem);

    CDSSubsystem = mCDSSubsystem;
    intakeSubsystem = mIntakeSubsystem;

    allianceColor = CDSSubsystem.getAllianceColor();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int ballCount = CDSSubsystem.getBallCount();
    String ballColor = CDSSubsystem.senseColor();
  
    switch (state) {
      case IDLE:
        CDSSubsystem.stopCDS();
        intakeSubsystem.stopIntake();
        autoEjectRunning.setBoolean(false);
        autoIntakeRunning.setBoolean(false);
        
        msCurrent = 0;

        if (ballCount > 2 || ballColor != allianceColor) {
          state = ManagementState.EJECT;
        }

        break;
      
      case EJECT:
        intakeSubsystem.toggleIntake(true);
        CDSSubsystem.CDSWheelToggle(true);
        autoEjectRunning.setBoolean(true);
        
        if (msCurrent >= ejectRuntime) {
          state = ManagementState.IDLE;
        } else {
          msCurrent += 20;
        }

        break;
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
