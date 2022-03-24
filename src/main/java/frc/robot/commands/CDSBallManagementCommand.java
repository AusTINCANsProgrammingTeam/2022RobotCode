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
import frc.robot.subsystems.ShooterSubsystem;

public class CDSBallManagementCommand extends CommandBase {
  
  // TODO: Add a ADVANCE state 
  public enum ManagementState {
    IDLE,
    EJECT,
    ADVANCE
  }

  /** Creates a new CDSBallManagementCommand. */
  private final CDSSubsystem CDSSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private ManagementState state = ManagementState.IDLE;
  private String allianceColor;

  private int msCurrent = 0;
  private int ejectRuntime = 650; // amount of time auto eject will run intake backwards for in ms
  private int nextOpenSensor = -1; // placeholder 
  private boolean[] sensorStatus;

  private ShuffleboardTab CDSTab = Shuffleboard.getTab("CDS Tab");
  private NetworkTableEntry autoEjectRunning = 
      CDSTab.add("Auto Eject Running", false).getEntry();
  private NetworkTableEntry autoIntakeRunning = 
      CDSTab.add("Auto Intake Running", false).getEntry();
  private NetworkTableEntry ballcountEntry = 
      CDSTab.add("Ball Count", 0).getEntry();
  private NetworkTableEntry CDSState = 
      CDSTab.add("CDS State", "IDLE").getEntry();

  public CDSBallManagementCommand(CDSSubsystem mCDSSubsystem, IntakeSubsystem mIntakeSubsystem, ShooterSubsystem mShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mCDSSubsystem);
    addRequirements(mIntakeSubsystem);
    addRequirements(mShooterSubsystem);

    CDSSubsystem = mCDSSubsystem;
    intakeSubsystem = mIntakeSubsystem;
    shooterSubsystem = mShooterSubsystem;

    allianceColor = CDSSubsystem.getAllianceColor();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = ManagementState.IDLE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int ballCount = CDSSubsystem.getBallCount();
    String ballColor = CDSSubsystem.senseColor();

    sensorStatus = CDSSubsystem.getSensorStatus();
    int openSensor = CDSSubsystem.getNextOpenSensor(sensorStatus);
  
    switch (state) {
      case IDLE:
        CDSSubsystem.stopCDS();
        intakeSubsystem.stopIntake();
        shooterSubsystem.runCargo(0.0);
        autoEjectRunning.setBoolean(false);
        autoIntakeRunning.setBoolean(false);
        
        msCurrent = 0;
        nextOpenSensor = -1;

        if (ballCount > 2 || (CDSSubsystem.getMissedColor() && CDSSubsystem.getMissedSensor())) {
          state = ManagementState.EJECT;
          break;
        }

        
        if (ballCount < 3 && openSensor != -1 && CDSSubsystem.getMissedSensor()) {
          state = ManagementState.ADVANCE;
          nextOpenSensor = openSensor;
          break;
        }

        break;
      
      case EJECT:
        intakeSubsystem.toggleIntake(true);
        CDSSubsystem.CDSWheelToggle(true);
        autoEjectRunning.setString("true");
        ballcountEntry.setNumber(ballCount);
        
        if (msCurrent >= ejectRuntime) {
          state = ManagementState.IDLE;
          CDSSubsystem.setMissedSensor(false);
          CDSSubsystem.setMissedColor(false);
        } else {
          msCurrent += 20;
        }

        break;
      
      case ADVANCE:
        CDSSubsystem.CDSWheelToggle(false);
        CDSSubsystem.CDSBeltToggle(false);
        shooterSubsystem.runCargo(Constants.reverseStopperWheelSpeed);
        autoIntakeRunning.setBoolean(true);
        ballcountEntry.setNumber(ballCount);

        sensorStatus = CDSSubsystem.getSensorStatus();

        if (sensorStatus[nextOpenSensor]) {
          state = ManagementState.IDLE;
          CDSSubsystem.setMissedSensor(false);
          CDSSubsystem.setMissedColor(false);
        }

        break;
    }
    CDSState.setString(state.toString());
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
