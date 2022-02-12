// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.hardware.MotorController;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class CDSSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController CDSBeltController;
  private MotorController CDSWheelControllerOne;
  private MotorController CDSWheelControllerTwo;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("SmartDashboard");
  private ShuffleboardTab CDSTab = Shuffleboard.getTab("CDS Tab");
  private NetworkTableEntry CDSWheelControllerDirection = CDSTab.add("CDS Wheel Direction", "Not Running").withPosition(1, 0).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  private NetworkTableEntry CDSBeltControllerDirection = CDSTab.add("CDS Belt Direction", "Not Running").withPosition(2, 0).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  private NetworkTableEntry CDSWheelControllerSpeed = CDSTab.add("CDS Wheel speed",0).withPosition(3,0).getEntry();
  private NetworkTableEntry CDSBeltControllerSpeed = CDSTab.add("CDS Belt speed",0).withPosition(4, 0).getEntry();

  public CDSSubsystem() {
    CDSBeltController = new MotorController("CDS Motor", Constants.CDSBeltID, 40);
    CDSWheelControllerOne = new MotorController("Wheel Motor Controller 1", Constants.CDSWheelControllerOneID, 40);
    CDSWheelControllerTwo = new MotorController("Wheel Motor Controller 2", Constants.CDSWheelControllerTwoID, 40);

    CDSWheelControllerTwo.getSparkMax().follow(CDSWheelControllerOne.getSparkMax(), true);
  }
  

  public void CDSBeltWheelControllerToggle(boolean reverse) {
    if (reverse) {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      CDSWheelControllerDirection.setString("Reverse");

      //SmartDashboard.putString("CDS Wheel Controller Direction", "Reverse");
      //SmartDashboard.putNumber("CDS Wheel Controller Speed", -Constants.CDSWheelControllerSpeed);
      
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      CDSBeltController.setIdleMode(IdleMode.kBrake);
      CDSBeltControllerDirection.setString("Reverse");

      //SmartDashboard.putString("CDS Belt Direction", "Reverse");
      //SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
    } else {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      CDSWheelControllerDirection.setString("Forward");
      
      //SmartDashboard.putString("CDS Wheel Controller Direction", "Forward");
      //SmartDashboard.putNumber("CDS Wheel Controller Speed", Constants.CDSWheelControllerSpeed);
      
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      CDSBeltControllerDirection.setString("Forward");
      
      //SmartDashboard.putString("CDS Belt Direction", "Forward");
      //SmartDashboard.putNumber("CDS Belt Speed", -Constants.CDSBeltSpeed);
    }
  }

  public void stopCDS() {
    CDSWheelControllerOne.getSparkMax().set(0.0);
    CDSBeltController.getSparkMax().set(0.0);
    //SmartDashboard.putNumber("CDS Belt Speed", 0.0);
  }
  @Override
    public void periodic() {
      CDSBeltControllerSpeed.setDouble(CDSBeltController.getEncoder().getVelocity());
      CDSWheelControllerSpeed.setDouble(CDSWheelControllerTwo.getEncoder().getVelocity());
  
    }
} //Don't delete, for main method.