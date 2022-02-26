// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.common.hardware.ColorSensorMuxed;

public class CDSSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController CDSBeltController;
  private MotorController CDSWheelControllerOne;
  private MotorController CDSWheelControllerTwo;
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
    CDSWheelControllerOne = new MotorController("Wheel Motor Controller 1", Constants.CDSWheelControllerOneID, 40);
    CDSWheelControllerTwo = new MotorController("Wheel Motor Controller 2", Constants.CDSWheelControllerTwoID, 40);

    CDSWheelControllerOne.setInverted(true);
    CDSWheelControllerTwo.getSparkMax().follow(CDSWheelControllerOne.getSparkMax(), true);

    CDSBeltController.setIdleMode(IdleMode.kBrake);
    //CDSWheelControllerOne.setIdleMode(IdleMode.kCoast);

    colorSensors = new ColorSensorMuxed(0, 1, 3);

    String allianceColor = DriverStation.getAlliance().toString();
    SmartDashboard.putString("Alliance Color", allianceColor);
  }

  public void CDSToggleAll(boolean reverse) {
    if (reverse){
      CDSWheelControllerOne.getSparkMax().set(-Constants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Direction", "Reverse");
      SmartDashboard.putNumber("CDS Wheel Speed", -Constants.CDSWheelControllerSpeed);

      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      CDSBeltController.setIdleMode(IdleMode.kBrake);
      SmartDashboard.putString("CDS Belt Direction", "Reverse");
      SmartDashboard.putNumber("CDS Belt Speed", -Constants.CDSBeltSpeed);
    } else {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Direction", "Forward");
      SmartDashboard.putNumber("CDS Wheel Speed", Constants.CDSWheelControllerSpeed);

      CDSBeltController.getSparkMax().set(Constants.CDSBeltSpeed);
      SmartDashboard.putString("CDS Belt Direction", "Forward");
      SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
    }
  }

  public void CDSWheelToggle(boolean reverse) {
    if (reverse) {
      CDSWheelControllerOne.getSparkMax().set(-Constants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Direction", "Reverse");
      SmartDashboard.putNumber("CDS Wheel Speed", -Constants.CDSWheelControllerSpeed);
  
    } else {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Direction", "Forward");
      SmartDashboard.putNumber("CDS Wheel Speed", Constants.CDSWheelControllerSpeed);
    }
  }

  public void CDSBeltToggle(boolean reverse) {
    if(reverse){
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
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

    SmartDashboard.putNumber("Front Sensor Proximity", sensorStatuses[2]);
    SmartDashboard.putNumber("Middle Sensor Proximity", sensorStatuses[1]);
    SmartDashboard.putNumber("Back Sensor Proximity", sensorStatuses[0]);

    boolean backStatus = sensorStatuses[0] > Constants.backSensorActivation;
    boolean middleStatus = sensorStatuses[1] > Constants.middleSensorActivation;
    boolean frontStatus = sensorStatuses[2] > Constants.frontSensorActivation;
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
    for (int i=0; i < sensorStatus.length-1; i++) {
      if (!sensorStatus[i]) {
        return i;
      }
    }
    return -1;
  }

  public String senseColor() {
    Color[] colors = colorSensors.getColors();

    // Only sensing colors for first sensor so that we can handle it when it's coming in and not dealing with any other complexities
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

  public String getAllianceColor() {
    return allianceColor;
  }

  public void periodic() {
    String ballColor = senseColor();
    SmartDashboard.putBoolean("Ball Color Match", ballColor == allianceColor);
  }
}
