package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BatterySimulation extends SubsystemBase {

  private double voltageToSubtract = 0.05;
  private BatterySubsystem batterySubsystem;
  private PDPSim pdpSim = new PDPSim(1);

  public BatterySimulation(BatterySubsystem batterySubsystem) {
    this.batterySubsystem = batterySubsystem;
    pdpSim.setVoltage(14.0);
  }

  @Override
  public void simulationPeriodic() {
    double now = Timer.getFPGATimestamp();
    if (now >= 0 && now < 5.0) {
      pdpSim.setVoltage(pdpSim.getVoltage() - voltageToSubtract);
      if (pdpSim.getVoltage() < Constants.minVoltageRedDouble) {
        System.out.println("Got to assert voltage");
      }
    } else if (now >= 5.0 && now < Constants.timeInSecondsHighCurrentRed + 5.0) {
      if (batterySubsystem.getHighCurrentTimer() > Constants.timeInSecondsHighCurrentRed) {
        System.out.println("Got to assert high current timer");
      }
    } else if (now >= Constants.timeInSecondsHighCurrentRed + 5.0
        && now < Constants.timeInSecondsGeneralRed + 5.0) {
      if (batterySubsystem.getGeneralTimer() > Constants.timeInSecondsGeneralRed) {
        System.out.println("Got to assert general timer");
      }
    }
    // Enable the robot after 10 seconds to test high current timer
    if (batterySubsystem.getGeneralTimer() > 10.0) {
      DriverStationSim.setEnabled(true);
    }

    // Forcefully sets current when robot is enabled to simulate use
    if (DriverStationSim.getEnabled() == true) {
      pdpSim.setCurrent(1, Constants.highBatteryCurrentThreshold + 1);
    } else {
      pdpSim.setCurrent(1, Constants.highBatteryCurrentThreshold - 1);
    }
  }
}
