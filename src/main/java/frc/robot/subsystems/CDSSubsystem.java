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
  private String allianceColor;
  private ColorSensorMuxed colorSensors;

  private boolean isReady = true; // Variable for whether CDS is ready for shooter action
  private int ballCount = 0;

  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry DCDSSpeed =
      operatorTab
          .add("CDS Speed", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(3, 1)
          .getEntry();
  private NetworkTableEntry BManualCDS =
      operatorTab
          .add("Manual CDS", 0)
          .withPosition(5, 1)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry();

  private ShuffleboardTab CDSTab = Shuffleboard.getTab("CDS Tab");
  private NetworkTableEntry CDSWheelControllerDirection =
      CDSTab.add("CDS Wheel Direction", "Not Running").withPosition(1, 0).getEntry();
  private NetworkTableEntry CDSBeltControllerDirection =
      CDSTab.add("CDS Belt Direction", "Not Running").withPosition(2, 0).getEntry();
  private NetworkTableEntry CDSWheelControllerSpeed =
      CDSTab.add("CDS Wheel speed", 0).withPosition(3, 0).getEntry();
  private NetworkTableEntry CDSBeltControllerSpeed =
      CDSTab.add("CDS Belt speed", 0).withPosition(4, 0).getEntry();
  private NetworkTableEntry ballManagementEnabled =
      CDSTab.add("Ball Management Enabled", true).withPosition(5, 0).getEntry();

  public CDSSubsystem() {
    // BManualCDS.setBoolean(Constants.); TODO: setup when manual cds toggle is merged
    if (Constants.DebugMode) {
      instantiateDebugTab();
    }
    CDSBeltController = new MotorController("CDS Motor", Constants.CDSBeltID);
    CDSBeltController.setInverted(true);
    CDSWheelControllerOne =
        new MotorController("Wheel Motor Controller 1", Constants.CDSWheelControllerOneID);
    CDSWheelControllerTwo =
        new MotorController("Wheel Motor Controller 2", Constants.CDSWheelControllerTwoID);

    CDSWheelControllerOne.setInverted(true);
    CDSWheelControllerTwo.follow(CDSWheelControllerOne, true);

    CDSBeltController.setIdleMode(IdleMode.kBrake);
    CDSWheelControllerOne.setIdleMode(IdleMode.kCoast);

    colorSensors = new ColorSensorMuxed(2, 1, 0); // front to back color sensor ports on new robotn

    String allianceColor = DriverStation.getAlliance().toString();
    SmartDashboard.putString("Alliance Color", allianceColor);
  }

  public void CDSToggleAll(boolean reverse) {
    if (reverse) {
      CDSWheelControllerOne.set(-Constants.CDSWheelControllerSpeed);
      DCDSSpeed.setDouble(-1);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Wheel Direction", "Reverse");
        SmartDashboard.putNumber("CDS Wheel Speed", -Constants.CDSWheelControllerSpeed);
      }

      CDSBeltController.set(-Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction", "Reverse");
        SmartDashboard.putNumber("CDS Belt Speed", -Constants.CDSBeltSpeed);
      }
    } else {
      DCDSSpeed.setDouble(1);
      CDSWheelControllerOne.set(Constants.CDSWheelControllerSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Wheel Direction", "Forward");
        SmartDashboard.putNumber("CDS Wheel Speed", Constants.CDSWheelControllerSpeed);
      }

      CDSBeltController.set(Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction", "Forward");
        SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
      }
    }
  }

  public void CDSWheelToggle(boolean reverse) {
    if (reverse) {
      CDSWheelControllerOne.set(-Constants.CDSWheelControllerSpeed);
      CDSWheelControllerDirection.setString("Reverse");
    } else {
      CDSWheelControllerOne.set(Constants.CDSWheelControllerSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Wheel Direction", "Forward");
        SmartDashboard.putNumber("CDS Wheel Speed", Constants.CDSWheelControllerSpeed);
      }
    }
  }

  public void CDSBeltToggle(boolean reverse) {
    DCDSSpeed.setDouble(-1);
    if (reverse) {
      CDSBeltController.set(-Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction", "Reverse");
        SmartDashboard.putNumber("CDS Belt Speed", -Constants.CDSBeltSpeed);
      }

    } else {
      DCDSSpeed.setDouble(1);
      CDSBeltController.set(Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction", "Forward");
        SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
      }
    }
  }

  public double getBeltSpeed() {
    return CDSBeltController.getSpeed();
  }

  public double getWheelSpeed() {
    return CDSWheelControllerOne.getSpeed();
  }

  public void stopCDS() {
    DCDSSpeed.setDouble(0);
    // stops all motors in the CDS
    CDSWheelControllerOne.set(0.0);
    CDSBeltController.set(0.0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("CDS Wheel Speed", 0.0);
      SmartDashboard.putNumber("CDS Belt Speed", 0.0);
    }
  }

  public void stopCDSWheel() {
    // Stops only the centering wheels
    CDSWheelControllerOne.set(0.0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("CDS Wheel Speed", 0.0);
    }
  }

  /*public boolean sensorsOnline() {
    boolean sensor0Online = picoSensors.isSensor0Connected();
    boolean sensor1Online = picoSensors.isSensor1Connected();
    boolean sensor2Online = picoSensors.isSensor2Connected();

    return sensor0Online && sensor1Online && sensor2Online;
  }*/

  public boolean[] getSensorStatus() {
    int[] sensorStatuses = colorSensors.getProximities();
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("Front Sensor Proximity", sensorStatuses[2]);
      SmartDashboard.putNumber("Middle Sensor Proximity", sensorStatuses[1]);
      SmartDashboard.putNumber("Back Sensor Proximity", sensorStatuses[0]);
    }

    boolean backStatus = sensorStatuses[0] > Constants.backSensorActivation;
    boolean middleStatus = sensorStatuses[1] > Constants.middleSensorActivation;
    boolean frontStatus = sensorStatuses[2] > Constants.frontSensorActivation;
    boolean[] beamBreakArray = {backStatus, middleStatus, frontStatus};

    ballCount = 0;
    for (boolean status : beamBreakArray) {
      if (status) {
        ballCount++;
      }
    }
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("Ball Count", ballCount);
    }

    return beamBreakArray;
  }

  public int getNextOpenSensor(boolean[] sensorStatus) {
    // Starts at 0 and ends short of the centering wheel
    for (int i = 0; i < sensorStatus.length - 1; i++) {
      if (!sensorStatus[i]) {
        return i;
      }
    }
    return -1;
  }

  public String senseColor() {
    Color[] colors = colorSensors.getColors();

    // Only sensing colors for first sensor so that we can handle it when it's coming in and not
    // dealing with any other complexities
    double redAmount = colors[2].red;
    double blueAmount = colors[2].blue;
    if (redAmount > blueAmount) {
      SmartDashboard.putString("Ball Color", "Red");
      return "Red";
    } else {
      SmartDashboard.putString("Ball Color", "Blue");
      return "Blue";
    }
  }

  /*public boolean managementEnabled() {
    return SmartDashboard.getBoolean("Ball Management Enabled", true);
  }*/

  public String getAllianceColor() {
    return allianceColor;
  }

  public int getBallCount() {
    return ballCount;
  }

  public boolean getReady() {
    return isReady;
  }

  public void setReady(boolean status) {
    isReady = status;
  }

  public void periodic() {}

  private void instantiateDebugTab() {
    CDSTab = Shuffleboard.getTab("CDS Tab");
    CDSBeltControllerDirection =
        CDSTab.add("CDS Belt Direction", "Not Running")
            .withPosition(2, 0)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
    CDSWheelControllerSpeed = CDSTab.add("CDS Wheel speed", 0).withPosition(3, 0).getEntry();
    CDSBeltControllerSpeed = CDSTab.add("CDS Belt speed", 0).withPosition(4, 0).getEntry();
  }
}
