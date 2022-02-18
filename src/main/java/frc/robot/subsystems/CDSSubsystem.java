// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;


import frc.robot.common.hardware.MotorController;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTableEntry;
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
  private ColorSensorV3 colorSensorOne;
  private DigitalInput backBeamBreak;
  private String allianceColor;

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

    colorSensorOne = new ColorSensorV3(Constants.colorSensorPort);
    backBeamBreak = new DigitalInput(Constants.initialBallSensorChannel);

    String allianceColor = DriverStation.getAlliance().toString();
    SmartDashboard.putString("Alliance Color", allianceColor);
  }
  

  public void CDSBeltWheelControllerToggle(boolean reverse) {
    if (reverse) {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      CDSWheelControllerDirection.setString("Reverse");

 
      
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      CDSBeltController.setIdleMode(IdleMode.kBrake);
      CDSBeltControllerDirection.setString("Reverse");

 
    } else {
      CDSWheelControllerOne.getSparkMax().set(Constants.CDSWheelControllerSpeed);
      CDSWheelControllerDirection.setString("Forward");
      
 
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      CDSBeltControllerDirection.setString("Forward");
      

    }
  }
    
  public int[] getSensorStatus() {
    int frontStatus = colorSensorOne.getProximity() > 800 ? 1: 0;
    int backStatus = backBeamBreak.get() ? 1: 0;
    int[] beamBreakArray = {frontStatus, backStatus};
    return beamBreakArray;
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

  public void stopCDS() {
    CDSWheelControllerOne.getSparkMax().set(0.0);
    CDSBeltController.getSparkMax().set(0.0);

    }

  @Override
    public void periodic() {
      CDSBeltControllerSpeed.setDouble(CDSBeltController.getEncoder().getVelocity());
      CDSWheelControllerSpeed.setDouble(CDSWheelControllerTwo.getEncoder().getVelocity());
      String ballColor = senseColor();
      SmartDashboard.putString("Ball Color", ballColor);
      SmartDashboard.putBoolean("Ball Color Match", ballColor == allianceColor);

      // Ball indexing
      int[] sensorStatus = getSensorStatus();
      int ballCount = sensorStatus[0] + sensorStatus[1];
      SmartDashboard.putNumber("Ball Count", ballCount);
  }
  


}

