package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants;

public class BatterySimulation extends SubsystemBase {
  /*
  Simulate voltage
  General timer - 2 test case
      - General Red
      - General Yellow
  High current timer - 2 test cases
      - High Current Red
      - High Current Yellow
  */
  private double voltageToSubtract = 0.0001;
  private BatterySubsystem batterySubsystem;

  public BatterySimulation(BatterySubsystem batterySubsystem) {
    this.batterySubsystem = batterySubsystem;
    RoboRioSim.setVInVoltage(14.0);
  }

  @Override
  public void simulationPeriodic() {
    //Consistently lowers the voltage
    RoboRioSim.setVInVoltage(RoboRioSim.getVInVoltage() - voltageToSubtract);



    //Checks voltage levels. If below a certain amount (minVoltageRedDouble), it crashes the simulation
    if (RoboRioSim.getVInVoltage() < Constants.minVoltageRedDouble) {
      System.out.println("Got to assert voltage");
      assertEquals(true, batterySubsystem.checkRedVoltage());
    }

    //Checks general timer. If a certain amount of time has passed (timeInSecondsGeneralRed), it crashes the simulation
    if (batterySubsystem.getGeneralTimer() > Constants.timeInSecondsGeneralRed) {
      System.out.println("Got to assert general timer");
      assertEquals(true, batterySubsystem.checkTimer());
    }
    //Checks high current timer. If a certain amount of time has passed (timeInSecondsHighCurrentRed), it crashes the simulation
    if (batterySubsystem.getHighCurrentTimer() > Constants.timeInSecondsHighCurrentRed) {
      System.out.println("Got to assert high current timer");
      assertEquals(true, batterySubsystem.checkTimer());
    }
    //Enable the robot after 10 seconds to rest high current timer
    if (batterySubsystem.getGeneralTimer() > 10.0 ) {
      DriverStationSim.setEnabled(true);
    }

    //Forcefully sets the voltage when enabling the robot.
    if (DriverStationSim.getAutonomous() == true || DriverStationSim.getEnabled() == true) {
      batterySubsystem.startHCTimer();
      //RoboRioSim.setVInVoltage(12.1);
    } else {
      //RoboRioSim.setVInVoltage(14.0);
      batterySubsystem.stopHCTimer();
    }
  }
}
