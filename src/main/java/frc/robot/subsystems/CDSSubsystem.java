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

  private int[] sensorStatuses;
  private int sensorsDown = 0;
  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry DCDSSpeed =
      operatorTab
          .add("CDS Speed", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(3, 1)
          .getEntry();

  private ShuffleboardTab CDSTab = Shuffleboard.getTab("CDS Tab");
  private NetworkTableEntry ballColor = 
      CDSTab.add("Ball Color", "blue").getEntry();
  private NetworkTableEntry CDSBallCount = 
      CDSTab.add("Ball Count", 0).getEntry();
  private NetworkTableEntry frontSensorProx = 
      CDSTab.add("Front Proximity", 0).getEntry();
  private NetworkTableEntry middleSensorProx = 
      CDSTab.add("Middle Proximity", 0).getEntry();
  private NetworkTableEntry backSensorProx = 
      CDSTab.add("Back Proximity", 0).getEntry();

  public CDSSubsystem() {
    // BManualCDS.setBoolean(Constants.); TODO: setup when manual cds toggle is merged
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

    colorSensors = new ColorSensorMuxed(1, 2, 0); // front to back color sensor ports on new robotn

    allianceColor = DriverStation.getAlliance().toString();
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
      SmartDashboard.putString("CDS Wheel Direction", "Reverse");
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
    if (Constants.DebugMode){
      frontSensorProx.setNumber(sensorStatuses[2]);
      middleSensorProx.setNumber(sensorStatuses[1]);
      backSensorProx.setNumber(sensorStatuses[0]);
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
      CDSBallCount.setNumber(ballCount);
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
      ballColor.setString("red");
      return "Red";
    } else {
      ballColor.setString("blue");
      return "Blue";
    }
  }

  public boolean sensorsOnline() {
    sensorStatuses = colorSensors.getProximities();
    for (int prox : sensorStatuses) {
      if (prox == 0) {
        sensorsDown += 1;
        return false;
      }
    }
    sensorsDown = 0;
    return true;
  }

  public int getSensorDown() {
    return sensorsDown;
  }

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
}
