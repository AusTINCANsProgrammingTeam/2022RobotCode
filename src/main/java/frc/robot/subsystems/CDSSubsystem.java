// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogTrigger;

/** Add your docs here. */
public class CDSSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController CDSBeltController;
  private MotorController CDSWheelControllerOne;
  private MotorController CDSWheelControllerTwo;
  private DigitalInput intitalBallSensor;
  private DigitalInput middleBallSensor;
  private DigitalInput finalBallSensor;
  private int ballCount = 0;
  
  public CDSSubsystem() {
    CDSBeltController = new MotorController("CDS Motor", Constants.CDSMotorThreeID);
    CDSWheelControllerOne =  new MotorController("Main CDS Wheel Controller", Constants.CDSMotorFourID);
    CDSWheelControllerTwo = new MotorController("Follows CDS Wheel Controller", Constants.CDSMotorFiveID);
    
    //intitalBallSensor = new DigitalInput(Constants.ballSensorChannel);
    //middleBallSensor = new DigitalInput(Constants.ballSensorChannel);
    //finalBallSensor = new DigitalInput(Constants.ballSensorChannel);

    CDSWheelControllerTwo.getSparkMax().follow(CDSWheelControllerOne.getSparkMax());
  }

  public void CDSSwitch(boolean on) {
    if (on) {
      double beltSmartSpeed = SmartDashboard.getNumber("Belt Speed", Constants.CDSBeltSpeed);
      double wheelSmartSpeed = SmartDashboard.getNumber("Wheel Speed", Constants.CDSWheelSpeed);

      CDSBeltController.getSparkMax().set(beltSmartSpeed);
      CDSWheelControllerOne.getSparkMax().set(wheelSmartSpeed);
      SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
      SmartDashboard.putNumber("CDS Wheel Speed", Constants.CDSWheelSpeed);
    } else {
      CDSBeltController.getSparkMax().set(0.0);
      CDSWheelControllerOne.getSparkMax().set(0.0);
      SmartDashboard.putNumber("CDS Motor Speed", 0);
    }
  }

  public void ForwardCDS() {
    CDSBeltController.getSparkMax().setInverted(false);
    CDSWheelControllerOne.getSparkMax().setInverted(false);
    SmartDashboard.putString("CDS Belt Direction", "Forward");
    SmartDashboard.putString("CDS Wheel Direction", "Forward");
  }

  public void ReverseCDS() {
    CDSBeltController.getSparkMax().setInverted(true);
    CDSWheelControllerOne.getSparkMax().setInverted(true);
    SmartDashboard.putString("CDS Belt Direction", "Reverse");
    SmartDashboard.putString("CDS Wheel Direction", "Reverse");
  }

  /*public boolean getDirection() {
    // true = inverted, false = forward
    return CDSBeltController.getSparkMax().getInverted();
  }

  public boolean getInitialSensorStatus(){
    return intitalBallSensor.get();
  }

  public boolean getMiddleSensorStatus(){
    return middleBallSensor.get();
  }

  public boolean getFinalSensorStatus(){
    return finalBallSensor.get();
  }

  public void indexBall() {
    boolean initialSensorStatus = getInitialSensorStatus();
    boolean finalSensorStatus = getFinalSensorStatus();
    if (!initialSensorStatus)  {
      if (!this.getDirection()){
        ballCount++;
      } else {
        ballCount--;
      }

      if (ballCount > 2) {
        this.ReverseCDS();
        // TODO: Determine how long to run this for (convert back to going normal direction)
      }
    } else if (finalSensorStatus == false) {
      ballCount--;
    }
  }

  public int getBallCount() {
    return ballCount;
  }*/
}