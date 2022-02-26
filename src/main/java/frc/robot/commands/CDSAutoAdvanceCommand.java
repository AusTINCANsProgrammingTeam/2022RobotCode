// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CDSSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CDSAutoAdvanceCommand extends CommandBase {
  /** Creates a new OuttakeCommand. */
  private final CDSSubsystem CDSSubsystem;
  private boolean runningCDS = false;
  private int setpointIndex;

  public CDSAutoAdvanceCommand(CDSSubsystem mCDSSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mCDSSubsystem);
    CDSSubsystem = mCDSSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean[] sensorStatus = CDSSubsystem.getSensorStatus();
    SmartDashboard.putBoolean("Front sensor status", sensorStatus[2]);
    SmartDashboard.putBoolean("Middle Sensor Status", sensorStatus[1]);
    SmartDashboard.putBoolean("Back Sensor Status", sensorStatus[0]);
    if (!runningCDS) {

      // Send ball to setpoint
      if (sensorStatus[2]) {  
        int nextOpenSensor = CDSSubsystem.getNextOpenSensor(sensorStatus);
        SmartDashboard.putNumber("Setpoint", nextOpenSensor);
        if (nextOpenSensor != -1) {
          // There is an open setpoint avaliable, run CDS
          runningCDS = true;
          setpointIndex = nextOpenSensor;
          CDSSubsystem.CDSToggleAll(false);
        }
      } 
    } else {
      // Check if ball has reached setpoint, stop if it has
      if (sensorStatus[setpointIndex]) {
        CDSSubsystem.stopCDS();
        runningCDS = false;
        setpointIndex = -1;
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

