// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.ColorSensorMuxed;
import frc.robot.common.hardware.MotorController;

public class CDSSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController CDSBeltController;
  private MotorController CDSWheelControllerOne;
  private MotorController CDSWheelControllerTwo;
  // private ColorSensorV3 colorSensorOne;
  // private ColorSensorV3 colorSensorTwo;
  // private DigitalInput backBeamBreak;
  private String allianceColor;
  private boolean runningCDS = false;
  private int setpointIndex;
  private ColorSensorMuxed colorSensors;

  private ShuffleboardTab CDSTab = Shuffleboard.getTab("CDS Tab");
  private NetworkTableEntry CDSWheelControllerDirection =
      CDSTab.add("CDS Wheel Direction", "Not Running")
          .withPosition(1, 0)
          .withWidget(BuiltInWidgets.kToggleSwitch)
          .getEntry();
  private NetworkTableEntry CDSBeltControllerDirection =
      CDSTab.add("CDS Belt Direction", "Not Running")
          .withPosition(2, 0)
          .withWidget(BuiltInWidgets.kToggleSwitch)
          .getEntry();
  private NetworkTableEntry CDSWheelControllerSpeed =
      CDSTab.add("CDS Wheel speed", 0).withPosition(3, 0).getEntry();
  private NetworkTableEntry CDSBeltControllerSpeed =
      CDSTab.add("CDS Belt speed", 0).withPosition(4, 0).getEntry();

  public CDSSubsystem() {
    CDSBeltController = new MotorController("CDS Motor", Constants.CDSBeltID, 40);
    CDSBeltController.setInverted(true);
    CDSWheelControllerOne =
        new MotorController("Wheel Motor Controller 1", Constants.CDSWheelControllerOneID, 40);
    CDSWheelControllerTwo =
        new MotorController("Wheel Motor Controller 2", Constants.CDSWheelControllerTwoID, 40);

    CDSWheelControllerTwo.getSparkMax().follow(CDSWheelControllerOne.getSparkMax(), true);

    colorSensors = new ColorSensorMuxed(0, 1, 2);

    // colorSensorOne = new ColorSensorV3(Constants.colorSensorPort1);
    // colorSensorTwo = new ColorSensorV3(Constants.colorSensorPort2);
    // backBeamBreak = new DigitalInput(Constants.initialBallSensorChannel);

    String allianceColor = DriverStation.getAlliance().toString();
    SmartDashboard.putString("Alliance Color", allianceColor);
  }

  public void CDSWheelToggle(boolean reverse) {
    if (reverse) {
      CDSWheelControllerOne.getSparkMax().set(-Constants.CDSWheelControllerSpeed);
      CDSWheelControllerDirection.setString("Reverse");

      CDSBeltController.getSparkMax().set(Constants.CDSBeltSpeed);
      CDSBeltController.setIdleMode(IdleMode.kBrake);
      CDSBeltControllerDirection.setString("Reverse");

    } else {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Direction", "Forward");
      SmartDashboard.putNumber("CDS Wheel Speed", Constants.CDSWheelControllerSpeed);
    }
  }

  public void CDSBeltToggle(boolean reverse) {
    if (reverse) {
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      CDSBeltController.setIdleMode(IdleMode.kBrake);
      SmartDashboard.putString("CDS Belt Direction", "Reverse");
      SmartDashboard.putNumber("CDS Belt Speed", -Constants.CDSBeltSpeed);

    } else {
      CDSBeltController.getSparkMax().set(Constants.CDSBeltSpeed);
      SmartDashboard.putString("CDS Belt Direction", "Forward");
      SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
    }
  }

  public double getBeltSpeed() {
    return CDSBeltController.getSpeed();
  }

  public double getWheelSpeed() {
    return CDSWheelControllerOne.getSpeed();
  }

  public void stopCDS() {
    // stops all motors in the CDS
    CDSWheelControllerOne.getSparkMax().set(0.0);
    CDSBeltController.getSparkMax().set(0.0);
    SmartDashboard.putNumber("CDS Wheel Speed", 0.0);
    SmartDashboard.putNumber("CDS Belt Speed", 0.0);
  }

  public void stopCDSWheel() {
    // Stops only the centering wheels
    CDSWheelControllerOne.getSparkMax().set(0.0);
    SmartDashboard.putNumber("CDS Wheel Speed", 0.0);
  }

  public boolean[] getSensorStatus() {
    int[] sensorStatuses = colorSensors.getProximities();

    boolean backStatus = sensorStatuses[2] > Constants.backSensorActivation;
    boolean middleStatus = sensorStatuses[1] > Constants.middleSensorActivation;
    boolean frontStatus = sensorStatuses[0] > Constants.frontSensorActivation;
    boolean[] beamBreakArray = {backStatus, middleStatus, frontStatus};

    int ballCount = 0;
    for (boolean status : beamBreakArray) {
      if (status) {
        ballCount++;
      }
    }

    SmartDashboard.putNumber("Ball Count", ballCount);

    return beamBreakArray;
  }

  public int getNextOpenSensor(boolean[] sensorStatus) {
    // Starts at 0 and ends short of the centering wheel
    for (int i = 0; i < sensorStatus.length - 1; i++) {
      if (sensorStatus[i]) {
        return i;
      }
    }
    return -1;
  }

  public void periodic() {
    // Color sensing
    String ballColor = senseColor();
    // SmartDashboard.putString("Ball Color", ballColor);
    SmartDashboard.putBoolean("Ball Color Match", ballColor == allianceColor);
    // boolean[] sensorStatus = getSensorStatus();

    /*
    // Send ball to setpoint
    if (!runningCDS) {
      SmartDashboard.putBoolean("Front sensor status", sensorStatus[2]);
      SmartDashboard.putBoolean("Middle Sensor Status", sensorStatus[1]);
      SmartDashboard.putBoolean("Back Sensor Status", sensorStatus[0]);

      if (sensorStatus[2]) {  //1 means sensor is activated
        int nextOpenSensor = getNextOpenSensor(sensorStatus);
        SmartDashboard.putNumber("Setpoint", nextOpenSensor);
        if (nextOpenSensor != -1) {
          // There is an open setpoint avaliable, run CDS
          runningCDS = true;
          setpointIndex = nextOpenSensor;
          CDSWheelToggle(false); // Run wheel
          CDSBeltToggle(false); // Run belt
        }
      }
    } else {
      // Check if ball has reached setpoint, stop if it has
      if (sensorStatus[setpointIndex]) {
        stopCDS();
        runningCDS = false;
        setpointIndex = -1;
      }
    }*/
  }

  /*
  public String getAllianceColor() {
    Alliance alliance = DriverStation.getAlliance();
    SmartDashboard.putString("Alliance Color", alliance.toString());
    return alliance.toString();
  }*/

  public String senseColor() {

    Color[] colors = colorSensors.getColors();

    // Only sensing colors for first sensor so that we can handle it when it's coming in and not
    // dealing with any other complexities
    double redAmount = colors[0].red;
    double blueAmount = colors[0].blue;
    if (redAmount > blueAmount) {
      SmartDashboard.putString("Ball Color", "Red");
      return "Red";
    } else {
      SmartDashboard.putString("Ball Color", "Blue");
      return "Blue";
    }
  }
}
