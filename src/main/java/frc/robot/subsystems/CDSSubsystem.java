// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DigitalInput;

public class CDSSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController CDSBeltController;
  private MotorController CDSWheelControllerOne;
  private MotorController CDSWheelControllerTwo;
  private ColorSensorV3 colorSensorOne;
  private ColorSensorV3 colorSensorTwo;
  private DigitalInput backBeamBreak;
  private String allianceColor;
  private boolean runningCDS = false;
  private int setpointIndex;

  public CDSSubsystem() {
    CDSBeltController = new MotorController("CDS Motor", Constants.CDSBeltID, 40);
    CDSWheelControllerOne = new MotorController("Wheel Motor Controller 1", Constants.CDSWheelControllerOneID, 40);
    CDSWheelControllerTwo = new MotorController("Wheel Motor Controller 2", Constants.CDSWheelControllerTwoID, 40);

    CDSWheelControllerTwo.getSparkMax().follow(CDSWheelControllerOne.getSparkMax(), true);

    colorSensorOne = new ColorSensorV3(Constants.colorSensorPort1);
    colorSensorTwo = new ColorSensorV3(Constants.colorSensorPort2);
    backBeamBreak = new DigitalInput(Constants.initialBallSensorChannel);

    String allianceColor = DriverStation.getAlliance().toString();
    SmartDashboard.putString("Alliance Color", allianceColor);
  }

  public void CDSWheelToggle(boolean reverse) {
    if (reverse) {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Controller Direction", "Reverse");
      SmartDashboard.putNumber("CDS Wheel Controller Speed", -Constants.CDSWheelControllerSpeed);
  
    } else {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Controller Direction", "Forward");
      SmartDashboard.putNumber("CDS Wheel Controller Speed", Constants.CDSWheelControllerSpeed);
      
    }
  }

  public void CDSBeltToggle(boolean reverse) {
    if(reverse){
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      CDSBeltController.setIdleMode(IdleMode.kBrake);
      SmartDashboard.putString("CDS Belt Direction", "Reverse");
      SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);

    } else {
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      SmartDashboard.putString("CDS Belt Direction", "Forward");
      SmartDashboard.putNumber("CDS Belt Speed", -Constants.CDSBeltSpeed);
    }
  }

  public void stopCDS() {
    // stops all motors in the CDS
    CDSWheelControllerOne.getSparkMax().set(0.0);
    CDSBeltController.getSparkMax().set(0.0);
    SmartDashboard.putNumber("CDS Speed", 0.0);
  }

  public void stopCDSWheel() {
    // Stops only the centering wheels
    CDSWheelControllerOne.getSparkMax().set(0.0);
    SmartDashboard.putNumber("CDS Wheel Speed", 0.0);
  }

  public void CDSEject() {
    // TODO: Add intake object to interface with
    // Reverse both intake and CDS motors
  }
  
  public int[] getSensorStatus() {
    int frontStatus = colorSensorOne.getProximity() > 800 ? 1: 0;
    int middleStatus = colorSensorTwo.getProximity() > 800 ? 1: 0;
    int backStatus = backBeamBreak.get() ? 1: 0;
    int[] beamBreakArray = {backStatus, middleStatus, frontStatus};
    return beamBreakArray;
  }

  public int getNextOpenSensor() {
    // Setpoint 1 @ Shooter Stopper Wheel
    // Setpoint 2 @ Second beam break
    int[] sensorStatus = getSensorStatus();
    
    // Start at 1 to skip over the centering wheel status 
    for (int i=1; i < sensorStatus.length; i++) {
      if (sensorStatus[i] == 0) {
        return i;
      } 
    }
    return -1;
  }

  public void periodic() {
    // Color sensing
    String ballColor = senseColor();
    SmartDashboard.putString("Ball Color", ballColor);
    SmartDashboard.putBoolean("Ball Color Match", ballColor == allianceColor);

    // Ball indexing
    int[] sensorStatus = getSensorStatus();
    int ballCount = sensorStatus[0] + sensorStatus[1] + sensorStatus[2];
    SmartDashboard.putNumber("Ball Count", ballCount);

    // Send ball to setpoint
    if (!runningCDS) {
      if (sensorStatus[2] == 1) {  //1 means sensor is activated
        int nextOpenSensor = getNextOpenSensor();

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
      if (sensorStatus[setpointIndex] == 1) {
        stopCDS();
        runningCDS = false;
        setpointIndex = -1;
      } else if ((sensorStatus[2] == 0) && (CDSWheelControllerOne.getSparkMax().get() != 0)) {
          stopCDSWheel();
      }
    }
  }

  /*
  public String getAllianceColor() {
    Alliance alliance = DriverStation.getAlliance();
    SmartDashboard.putString("Alliance Color", alliance.toString());
    return alliance.toString();
  }*/

  public String senseColor() {
    Color color = colorSensorOne.getColor();
    double redAmount = color.red;
    double blueAmount = color.blue;
    if (redAmount > blueAmount) {
      SmartDashboard.putString("Ball Color", "Red");
      return "Red"; 
    } else {
      SmartDashboard.putString("Ball Color", "Blue");
      return "Blue";
    } 
  }
}