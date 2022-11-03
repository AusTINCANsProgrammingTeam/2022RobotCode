package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BatterySimulation extends SubsystemBase {

  private double voltageToSubtract = 0.05;
  private BatterySubsystem batterySubsystem;

  public BatterySimulation(BatterySubsystem batterySubsystem) {
    this.batterySubsystem = batterySubsystem;
    RoboRioSim.setVInVoltage(14.0);
  }

  @Override
  public void simulationPeriodic() {
    double now = Timer.getFPGATimestamp();
    if (now >= 0 && now < 5.0) {
      RoboRioSim.setVInVoltage(RoboRioSim.getVInVoltage() - voltageToSubtract);
      if (RoboRioSim.getVInVoltage() < Constants.minVoltageRedDouble) {
        System.out.println("Got to assert voltage");
        assertEquals(false, batterySubsystem.checkRedVoltage());
      }
    } else if (now >= 5.0 && now < 25.0) {
      if (batterySubsystem.getGeneralTimer() > Constants.timeInSecondsGeneralRed) {
        System.out.println("Got to assert general timer");
        assertEquals(true, batterySubsystem.checkTimer());
      }
    } else if (now >= 25.0 && now < 50.0) {
      if (batterySubsystem.getHighCurrentTimer() > Constants.timeInSecondsHighCurrentRed) {
        System.out.println("Got to assert high current timer");
        assertEquals(true, batterySubsystem.checkTimer());
      }
    }
    // Enable the robot after 10 seconds to test high current timer
    if (batterySubsystem.getGeneralTimer() > 10.0) {
      DriverStationSim.setEnabled(true);
    }

    // Forcefully sets current when robot is enabled to simulate use
    if (DriverStationSim.getAutonomous() == true || DriverStationSim.getEnabled() == true) {
      RoboRioSim.setVInCurrent(Constants.simCurrentHigh);
    } else {
      RoboRioSim.setVInCurrent(Constants.simCurrentLow);
    }
  }
}
