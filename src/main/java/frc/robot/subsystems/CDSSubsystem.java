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

/** Add your docs here. */
public class CDSSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController CDSBeltController;
  private MotorController CDSWheelControllerOne;
  private MotorController CDSWheelControllerTwo;
  private DigitalInput frontBallSensor;
  private DigitalInput middleBallSensor;
  private DigitalInput finalBallSensor;

  public CDSSubsystem() {
    CDSBeltController = new MotorController("CDS Motor", Constants.CDSBeltID, 40);
    CDSWheelControllerOne = new MotorController("Wheel Motor Controller 1", Constants.CDSWheelControllerOneID, 40);
    CDSWheelControllerTwo = new MotorController("Wheel Motor Controller 2", Constants.CDSWheelControllerTwoID, 40);

    CDSWheelControllerTwo.getSparkMax().follow(CDSWheelControllerOne.getSparkMax(), true);

    frontBallSensor = new DigitalInput(Constants.initialBallSensorChannel);
    middleBallSensor = new DigitalInput(Constants.middleBallSensorChannel);
    finalBallSensor = new DigitalInput(Constants.finalBallSensorChannel); 
  }

  public void CDSBeltWheelControllerToggle(boolean reverse) {
    if (reverse) {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Controller Direction", "Reverse");
      SmartDashboard.putNumber("CDS Wheel Controller Speed", -Constants.CDSWheelControllerSpeed);
      
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      CDSBeltController.setIdleMode(IdleMode.kBrake);
      SmartDashboard.putString("CDS Belt Direction", "Reverse");
      SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
    } else {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Controller Direction", "Forward");
      SmartDashboard.putNumber("CDS Wheel Controller Speed", Constants.CDSWheelControllerSpeed);
      
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      SmartDashboard.putString("CDS Belt Direction", "Forward");
      SmartDashboard.putNumber("CDS Belt Speed", -Constants.CDSBeltSpeed);
    }
  }

  public void stopCDS() {
    CDSWheelControllerOne.getSparkMax().set(0.0);
    CDSBeltController.getSparkMax().set(0.0);
    SmartDashboard.putNumber("CDS Belt Speed", 0.0);
  }
  
  public boolean[] getBeamBreakStatus() {
    boolean frontStatus = frontBallSensor.get();
    boolean middleStatus = middleBallSensor.get();
    boolean finalStatus = finalBallSensor.get();
    boolean[] beamBreakArray = {frontStatus, middleStatus, finalStatus};
    return beamBreakArray;
  }
  
  public void periodic() {
    SmartDashboard.putBooleanArray("Beam Break", getBeamBreakStatus());
  }

  public Alliance getAllianceColor() {
    Alliance alliance = DriverStation.getAlliance();
    SmartDashboard.putString("Alliance Color", alliance.toString());
    return alliance;
  }

  public Color senseColor() {
    ColorSensorV3 colorSensor = new ColorSensorV3(Constants.colorSensorPort);
    Color color = colorSensor.getColor();
    return color;
    
  }

} //Don't delete, for main method.