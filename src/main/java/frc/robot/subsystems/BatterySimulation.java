package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BatterySimulation extends SubsystemBase{
/*
    Simulate voltage
    General timer - 2 test case
        - General Red
        - General Yellow
    High current timer - 2 test cases
        - High Current Red
        - High Current Yellow
    */
    private double voltageToSubtract = 0.1;
    private BatterySubsystem batterySubsystem;
    

    public BatterySimulation(BatterySubsystem batterySubsystem) {
        this.batterySubsystem = batterySubsystem;
        RoboRioSim.setVInVoltage(14.0);
    }

    @Override
    public void simulationPeriodic() {
        RoboRioSim.setVInVoltage(RoboRioSim.getVInVoltage()-voltageToSubtract);
        if(RoboRioSim.getVInVoltage() < Constants.minVoltageRed) {
          assertEquals(true, batterySubsystem.checkRedVoltage());
        }
    }
    
}