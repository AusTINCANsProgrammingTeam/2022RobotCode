// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.*;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.Map;
/* Audio imports, not needed right now
import java.io.File;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.swing.JOptionPane;
*/
/** Add your docs here. */
// Put methods for controlling this subsystem
// here. Call these from Commands.
public class BatterySubsystem extends SubsystemBase {

  private ShuffleboardTab btTab;
  private NetworkTableEntry sbVoltage;
  private NetworkTableEntry sbInputCurrent;
  private NetworkTableEntry sbSimVoltage;
  private NetworkTableEntry sbTimer;
  private NetworkTableEntry sbTimerChange;
  private NetworkTableEntry sbTimerHighCurrent;
  private Timer timer = new Timer();
  private Timer currentTimer = new Timer();
  //private PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private PowerDistribution powerDistribution = new PowerDistribution();

  public BatterySubsystem() {
    this.resetTimers();
    btTab = Shuffleboard.getTab("Battery");
    Shuffleboard.getTab("Battery")
        .addBoolean("Battery Voltage Check", () -> getVoltage() > Constants.minVoltageRed)
        .withProperties(Map.of("colorWhenTrue", "red"))
        .withProperties(Map.of("colorWhenFalse", "red"));
    sbVoltage =
        btTab.add("Battery Voltage", 0).withSize(2, 1).withPosition(0, 0).getEntry();
    sbInputCurrent =
        btTab.add("Battery Input Current", 0).withSize(2, 1).withPosition(0, 0).getEntry();
    sbSimVoltage = btTab.add("Simulation Voltage", 0).withSize(2, 1).withPosition(2, 0).getEntry();
    sbTimer = btTab.add("Timer", 0).withSize(2, 1).withPosition(0, 2).getEntry();
    sbTimerHighCurrent =
        btTab.add("High Current Timer", 0).withSize(2, 1).withPosition(0, 0).getEntry();
    sbTimerChange = btTab.add("Change Timer", 0).withSize(2, 1).withPosition(4, 0).getEntry();
    timer.start();
    DriverStationSim.setSendError(true);
  }

  public void periodic() {
    sbVoltage.setDouble(getVoltage());
    sbInputCurrent.setDouble(getInputCurrent());
    sbSimVoltage.setNumber(powerDistribution.getVoltage());
    sbTimer.setDouble(timer.get());
    sbTimerHighCurrent.setDouble(currentTimer.get());
    sbTimerChange.setBoolean(checkTimer()); // Replace checkVoltage() with checkTimer() if necessary
    checkCurrent();
    checkVoltage();
  }

  public double getVoltage() {
      return powerDistribution.getVoltage();
  }

  public double getInputCurrent() {
    if (Robot.isSimulation()) {
      return powerDistribution.getCurrent(1);
    } else {
      return powerDistribution.getTotalCurrent();
    }
  }

  public void checkCurrent() {
    if (getInputCurrent() < Constants.maxBatteryCurrent) {
      stopHCTimer();
    } else {
      startHCTimer();
    }
  }

  public boolean checkTimer() {
    if (currentTimer.hasElapsed(Constants.timeInSecondsHighCurrentRed)) {
      DriverStation.reportWarning("Change the Battery Now! (HCTR)", false);
      return true;
    } else if (timer.hasElapsed(Constants.timeInSecondsGeneralRed)) {
      DriverStation.reportWarning("Change the Battery Now! (GTR)", false);
      // playAudio(Constants.audiofilepath);
      return true;
    } else if (currentTimer.hasElapsed(Constants.timeInSecondsHighCurrentYellow)) {
      DriverStation.reportWarning("Change the Battery Soon! (HCTY)", false);
      // playAudio(Constants.audiofilepath);
      return false;
    } else if (timer.hasElapsed(Constants.timeInSecondsGeneralYellow)) {
      DriverStation.reportWarning("Change the Battery Soon! (GTY)", false);
      // playAudio(Constants.audiofilepath);
      return false;
    }
    return false;
  }


  public int getMinute() {
    return (int) Math.floor(timer.get() / 60.0);
  }

  public double getGeneralTimer() {
    return timer.get();
  }

  public double getHighCurrentTimer() {
    return currentTimer.get();
  }

  public void startHCTimer() {
    currentTimer.start();
  }

  public void stopHCTimer() {
    currentTimer.stop();
  }

  public void checkVoltage() {
    if (getVoltage() < Constants.minVoltageRed) {
      DriverStation.reportWarning("Change the Battery Now! (VR)", false);
    } else if (getVoltage() < Constants.minVoltageYellow) {
      DriverStation.reportWarning("Change the Battery Soon! (VY)", false);
    } else {
    }
  }

  public boolean checkRedVoltage() {
    if (getVoltage() > Constants.minVoltageRed) {
      return true;
    } else {
      return false;
    }
  }
  public void resetTimers() {
    timer.reset();
    currentTimer.reset();
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
  /* This playaudio is an interesting way to remind people to change the battery, but it may require more
  work than it is worth.

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
  */
  //public void updateSmartDashboard() {}

  //TODO: Find a way to make these not reset every redeploy. Possibly make them save their values to a file, then
  // retrieve that file when the robot starts.
 
}
