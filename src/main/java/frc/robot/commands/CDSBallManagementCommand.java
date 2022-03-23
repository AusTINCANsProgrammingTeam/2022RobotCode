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
    public enum ManagementState {
    IDLE,
    EJECT,
    ADVANCE,
  }

  /** Creates a new CDSBallManagementCommand. */
  private final CDSSubsystem CDSSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private ManagementState state = ManagementState.IDLE;
  private String allianceColor;

  private int msCurrent = 0;
  private int ejectRuntime = 750;
  private int nextOpenSensor = -1; 
  private boolean[] sensorStatus;

  private ShuffleboardTab CDSTab = Shuffleboard.getTab("CDS Tab");
  private NetworkTableEntry autoEjectRunning = 
      CDSTab.add("Auto Eject Running", false).getEntry();
  private NetworkTableEntry autoIntakeRunning = 
      CDSTab.add("Auto Intake Running", false).getEntry();

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
  public void initialize() {}

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
        autoEjectRunning.setBoolean(false);
        autoIntakeRunning.setBoolean(false);
        
        msCurrent = 0;
        nextOpenSensor = -1;

        if (ballCount > 2 || ballColor != allianceColor) {
          state = ManagementState.EJECT;
        }

        if (ballCount < 2 && openSensor != -1) {
          state = ManagementState.ADVANCE;
          nextOpenSensor = openSensor;
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

      case ADVANCE:
        CDSSubsystem.CDSWheelToggle(false);
        CDSSubsystem.CDSBeltToggle(false);
        shooterSubsystem.runCargo(Constants.reverseStopperWheelSpeed);
        autoIntakeRunning.setBoolean(true);
      
        sensorStatus = CDSSubsystem.getSensorStatus();

        if (sensorStatus[nextOpenSensor]) {
          state = ManagementState.IDLE;
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
