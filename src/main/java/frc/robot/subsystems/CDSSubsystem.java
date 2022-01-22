// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class CDSSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController CDSBeltController;
  private MotorController CDSMotorController2;

  private DigitalInput intitalBallSensor;
  private DigitalInput middleBallSensor;
  private DigitalInput finalBallSensor;
  private int ballCount = 0;
 
  //TODO: Figure out what CDS motor is for
  public CDSSubsystem() {
    CDSBeltController = new MotorController("CDS Motor", Constants.CDSBeltID);
    CDSMotorController2 = new MotorController("CDS Motor 2", Constants.CDSMotorID);
   
    intitalBallSensor = new DigitalInput(Constants.ballSensorChannel);
    middleBallSensor = new DigitalInput(Constants.ballSensorChannel);
    finalBallSensor = new DigitalInput(Constants.ballSensorChannel);
  }

  public void CDSSwitch(boolean on) {
    if (on) {
      CDSBeltController.getSparkMax().set(Constants.CDSBeltSpeed);
      SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
    } else {
      CDSBeltController.getSparkMax().set(0.0);
      SmartDashboard.putNumber("CDS Motor Speed", 0);
    }
  }

  public void ForwardCDS() {
    CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
    SmartDashboard.putString("CDS Belt Direction", "Forward");
  }

  public void ReverseCDS() {
    CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
    SmartDashboard.putString("CDS Belt Direction", "Reverse");
  }

  public boolean getDirection() {
    // true = inverted, false = forward
    if (CDSBeltController.getSparkMax().get() > 0) {
      return false;
    } else {
    return CDSBeltController.getSparkMax().getInverted();
    }
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
  }
}