// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.*;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import java.io.File;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.swing.JOptionPane;
import java.util.Map;

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
  // private DriverStation driverStation;
  private Timer timer = new Timer();
  private Timer currentTimer = new Timer();
  //public DriverStationSim driverSim;
  // Sim variables
  private double simVolt;
  private double simCurrent;
  private boolean colorOn; //Variable for color change testing
  //private int minutesPassed;

  public BatterySubsystem() {
    // init();
    this.resetTimers();
    btTab = Shuffleboard.getTab("Battery");
    //Change 16 to desired value
    Shuffleboard.getTab("Battery").addBoolean("Battery Voltage", () -> getVoltage() > Constants.minVoltageRed).withProperties(Map.of("colorWhenTrue","red")).withProperties(Map.of("colorWhenFalse","red"));
    sbVoltage = btTab.add("Current Battery Voltage", 0).withSize(2, 2).withPosition(0, 0).getEntry();
    sbInCurrent = btTab.add("Input Current", 0).withSize(2, 2).withPosition(2, 0).getEntry();
    sbTimer = btTab.add("Timer", 0).withSize(2, 2).withPosition(0, 2).getEntry();
    sbTimerHighCurrent = btTab.add("High Current Timer", 0).withSize(2, 2).withPosition(0, 4).getEntry();
    sbTimerChange = btTab.add("Change Timer", 0).withSize(2, 2).withPosition(4, 0).getEntry();
    timer.start();
    currentTimer.start();
    simVolt = 16.0;
    DriverStationSim.setSendError(true);
    //btTab.addBoolean("Change the battery: ", valueSupplier)
  }

  public void periodic() {
    sbVoltage.setDouble(getVoltage());
    // sbInCurrent.setDouble(getInputCurrent());
    sbInCurrent.setNumber(getInputCurrent()); // Just temporary, replace with line above
    sbTimer.setDouble(timer.get());
    sbTimerHighCurrent.setDouble(currentTimer.get());
    sbTimerChange.setBoolean(checkTimer()); // Replace checkVoltage() with checkTimer() if necessary
    checkCurrent();
    checkVoltage();
    //checkMinute();
  }

  public double getVoltage() {
    if (Robot.isSimulation()) {
      return (int) simVolt;
    } else {
      return RobotController.getBatteryVoltage();
    }
  }

  public double getInputCurrent() {
    return RobotController.getInputCurrent();
  }

  public void checkCurrent() {
    if (getInputCurrent() < Constants.maxBatteryCurrent) {
      currentTimer.stop(); // Was current timer, but I got a warning about it needing to be static so I
      // changed it to this. Will this cause conflicts?
    } else {
      currentTimer.start();
    }
  }

  public boolean checkTimer() {
    if (currentTimer.hasElapsed(Constants.timeInSecondsHighCurrentRed)) {
      DriverStation.reportError("Change the Battery!", true);
      // playAudio(Constants.audiofilepath);
      return true;
    } else if (currentTimer.hasElapsed(Constants.timeInSecondsGeneralRed)) {
      DriverStation.reportError("Change the Battery!", true);
      // playAudio(Constants.audiofilepath);
      return true;
    } else if (timer.hasElapsed(Constants.timeInSecondsHighCurrentYellow)) {
      DriverStation.reportError("Change the Battery Soon!", true);
      // playAudio(Constants.audiofilepath);
      return false;
    } else if (timer.hasElapsed(Constants.timeInSecondsGeneralYellow)) {
      DriverStation.reportError("Change the Battery Soon!", true);
      // playAudio(Constants.audiofilepath);
      return false;
    }
    return false;
  }

  /*
  public void checkMinute() {
    double check = (getTimer() % 60.0);
    if (check > 0 & check < 0.025 & getTimer() > 1.0) {
      minutesPassed += 1;
    } else {
    }
  }
  */

  public int getMinute() {
    return (int) Math.floor(timer.get()/60.0);
  }

  public void checkVoltage() {
    if (getVoltage() < Constants.minVoltageRed) {
      DriverStation.reportError("Change the Battery!", true);
    } else if (getVoltage() < Constants.minVoltageYellow) {
      DriverStation.reportError("Change the Battery Soon!", true);
    } else {
    }
  }

  public boolean checkRedVoltage() {
    if(getVoltage() > Constants.minVoltageRed) {
      return true;
    } else {
      return false;
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
  static void playAudio(String location) {
    try {
      File path = new File(location);
      if (path.exists()) {
        AudioInputStream audioInput = AudioSystem.getAudioInputStream(path);
        Clip clip = AudioSystem.getClip();
        clip.open(audioInput);
        clip.start();
        clip.loop(2);

        JOptionPane.showMessageDialog(null, "Change the battery!");
        System.out.println("Done");

      } else {
        System.out.println("No file detected");
      }
    } catch (Exception ex) {
      ex.printStackTrace();
    }
  }

  public void updateSmartDashboard() {}

  public void resetTimers() {
    timer.reset();
    currentTimer.reset();
    //minutesPassed = 0;
  }

  /*
  @Override
  public void simulationPeriodic() {
    if (DriverStationSim.getAutonomous() == true || DriverStationSim.getEnabled() == true) {
      // Change voltage and current
      simVolt = 10.0;
    } else {
      simVolt = 15.0;
    }
  }
  */
}
