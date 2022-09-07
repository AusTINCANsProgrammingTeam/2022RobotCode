// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.*;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
// Put methods for controlling this subsystem
// here. Call these from Commands.
public class BatterySubsystem extends SubsystemBase {

  private ShuffleboardTab btTab;
  private NetworkTableEntry sbVoltage;
  private NetworkTableEntry sbInCurrent;
  private NetworkTableEntry sbTimer;
  private NetworkTableEntry sbTimerChange;
  private Timer timer;
  private int currentCounter;

  private void init() {
    timer.reset();
    btTab = Shuffleboard.getTab("Battery");
    sbVoltage = btTab.add("Battery Voltage", 0).withSize(2, 2).withPosition(0, 0).getEntry();
    sbInCurrent = btTab.add("Input Current", 0).withSize(2, 2).withPosition(2, 0).getEntry();
    sbTimer = btTab.add("Timer", 0).withSize(2, 2).withPosition(0, 2).getEntry();
    sbTimer = btTab.add("Change Timer", 0).withSize(2, 2).withPosition(2, 2).getEntry();
    timer.start();

    currentCounter = 0;
  }

  public BatterySubsystem() {
    init();
  }

  public void periodic() {
    sbVoltage.setDouble(getVoltage());
    sbInCurrent.setDouble(getInputCurrent());
    sbTimer.setDouble(getTimer());
    sbTimerChange.setBoolean(
        checkBattery()); 

    if (RobotController.getInputCurrent() >= Constants.maxCurrent) {
      currentCounter =+ 20;
    }
  }

  public double getVoltage() {
    return RobotController.getBatteryVoltage();
  }

  public double getInputCurrent() {
    return RobotController.getInputCurrent();
  }

  public double getTimer() {
    return Timer.getFPGATimestamp();
  }

  public boolean checkTimer() {
    if (timer.hasElapsed(Constants.seconds)) {
      return true;
    }
    return false;
  }

  public boolean checkVoltage() {
    if (getVoltage() < Constants.minVoltage) {
      return true;
    }
    return false;
  }

  public boolean checkCurrent() {
    if (currentCounter >= Constants.count) {
      return true;
    }
    return false;
  }

  public boolean checkBattery() {
    if (checkTimer()) {
      return true;
    } else if (checkVoltage()) {
      return true;
    } else if (checkCurrent()) {
      return true;
    }
    return false;
  }
   
  public void updateSmartDashboard() {}
}
