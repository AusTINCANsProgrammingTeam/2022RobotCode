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
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
// Put methods for controlling this subsystem
// here. Call these from Commands.
public class BatterySubsystem extends SubsystemBase {

  private ShuffleboardTab btTab;
  private NetworkTableEntry sbVoltage;
  private NetworkTableEntry sbInCurrent;
  private NetworkTableEntry sbTimer;
  private NetworkTableEntry sbTimerChange;
  private NetworkTableEntry sbTimerHighCurrent;
  private DriverStation driverStation;
  private Timer timer = new Timer();
  private Timer currentTimer = new Timer();

  private void init() {
    //timer.reset(); Only uncomment this if you want the timer to reset each time the robot redeploys code
    btTab = Shuffleboard.getTab("Battery");
    sbVoltage = btTab.add("Battery Voltage", 0).withSize(2, 2).withPosition(0, 0).getEntry();
    sbInCurrent = btTab.add("Input Current", 0).withSize(2, 2).withPosition(2, 0).getEntry();
    sbTimer = btTab.add("Timer", 0).withSize(2, 2).withPosition(0, 2).getEntry();
    sbTimerHighCurrent = btTab.add("High Current Timer", 0).withSize(2, 2).withPosition(0, 4).getEntry();
    sbTimerChange = btTab.add("Change Timer", 0).withSize(2, 2).withPosition(4, 0).getEntry();
    timer.start();
    currentTimer.start();
  }

  public BatterySubsystem() {
    init();
  }

  public void periodic() {
    sbVoltage.setDouble(getVoltage());
    sbInCurrent.setDouble(getInputCurrent());
    sbTimer.setDouble(getTimer());
    sbTimerHighCurrent.setDouble(getTimer());
    sbTimerChange.setBoolean(
        checkTimer()); // Replace checkVoltage() with checkTimer() if necessary
    checkCurrent();
  }

  public double getVoltage() {
    return RobotController.getBatteryVoltage();
  }

  public double getInputCurrent() {
    return RobotController.getInputCurrent();
  }
  public void checkCurrent() {
    if(getInputCurrent() < Constants.maxBatteryCurrent) {
      Timer.delay(0.02); //Was current timer, but I got a warning about it needing to be static so I changed it to this. Will this cause conflicts?
    }
  }


  public double getTimer() {
    return Timer.getFPGATimestamp();
  }

  public boolean checkTimer() {
    if (currentTimer.hasElapsed(Constants.timeInSecondsHighCurrentRed)) {
      DriverStation.reportError("Change the Battery!", true);
      return true;
    } else if (currentTimer.hasElapsed(Constants.timeInSecondsGeneralRed)) {
      DriverStation.reportError("Change the Battery!", true);
      return true;
    } else if (timer.hasElapsed(Constants.timeInSecondsHighCurrentYellow)) {
      DriverStation.reportError("Change the Battery Soon!", true);
      return false;
    } else if (timer.hasElapsed(Constants.timeInSecondsGeneralYellow)) {
      DriverStation.reportError("Change the Battery Soon!", true);
      return false;
    }
    return false;
  }

  public void checkVoltage() {
    if (getVoltage() < Constants.minVoltageRed) {
      DriverStation.reportError("Change the Battery!", true);
    } else if (getVoltage() < Constants.minVoltageYellow) {
      DriverStation.reportError("Change the Battery Soon!", true);
    }
    
  }
/* Old checkBattery function
  public boolean checkBattery() {
    if (checkTimer()) {
      return true;
    } else if (checkVoltage() < Constants.min) {
      return true;
    } else if (checkCurrent()) {
      return true;
    }
    return false;
  }
*/
  public void updateSmartDashboard() {}

public void resetTimers() {
  timer.reset();
  currentTimer.reset();
}
}
