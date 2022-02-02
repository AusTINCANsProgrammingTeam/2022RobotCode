// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private MotorController intakeMotorControllerOne;
  private MotorController intakeMotorControllerTwo;
  //private MotorController CDSWheelControllerOne;
  //private MotorController CDSWheelControllerTwo;
  private DigitalInput frontBallSensor;
  private DigitalInput middleBallSensor;
  private DigitalInput finalBallSensor;

  private int ballCount = 0;

  public IntakeSubsystem() {
    intakeMotorControllerOne = new MotorController("Intake Motor One", Constants.intakeMotorOneID, 40);
    intakeMotorControllerTwo = new MotorController("Intake Motor Two", Constants.intakeMotorTwoID, 40);
    //CDSWheelControllerOne = new MotorController("Wheel Motor Controller 1", Constants.intakeWheelOneID, 40);
    //CDSWheelControllerTwo = new MotorController("Wheel Motor Controller 2", Constants.intakeWheelTwoID, 40);

    frontBallSensor = new DigitalInput(Constants.initialBallSensorChannel);
    middleBallSensor = new DigitalInput(Constants.middleBallSensorChannel);
    finalBallSensor = new DigitalInput(Constants.finalBallSensorChannel); 

    DigitalInput[] sensorArray = {frontBallSensor, middleBallSensor, finalBallSensor};

    // Remove invert=true parameter if wheels aren't running correctly
    //CDSWheelControllerOne.getSparkMax().follow(intakeMotorControllerOne.getSparkMax());
    //CDSWheelControllerTwo.getSparkMax().follow(intakeMotorControllerOne.getSparkMax(), true);
    intakeMotorControllerTwo.getSparkMax().follow(intakeMotorControllerOne.getSparkMax());
  }

  public void toggleIntake(boolean reverse) {
    if (reverse) {
      intakeMotorControllerOne.getSparkMax().set(-Constants.intakeMotorSpeed);
      SmartDashboard.putString("Intake Motor Direction", "Reverse");
      SmartDashboard.putNumber("Intake Motor Speed", -Constants.intakeMotorSpeed);
    } else {
      intakeMotorControllerOne.getSparkMax().set(Constants.intakeMotorSpeed);
      SmartDashboard.putString("Intake Motor Direction", "Forward");
      SmartDashboard.putNumber("Intake Motor Speed", Constants.intakeMotorSpeed);
    }
  }

  public void stopIntake() {
    intakeMotorControllerOne.getSparkMax().set(0.0);
    SmartDashboard.putNumber("Intake Motor Speed", 0.0);
  }

  public boolean getDirection() {
    // true = inverted, false = forward
    if (intakeMotorControllerOne.getSparkMax().get() > 0) {
      return false;
    } else {
    return true;
    }
  }

  public boolean[] getBeamBreakStatus() {
    boolean frontStatus = frontBallSensor.get();
    boolean middleStatus = middleBallSensor.get();
    boolean finalStatus = finalBallSensor.get();
    boolean[] beamBreakArray = {frontStatus, middleStatus, finalStatus};
    return beamBreakArray;
  }

  public int getBallCount() {
    boolean[] beamBreakStatuses = this.getBeamBreakStatus();
    // TODO: Possibly add more beambreaks once done with testing.
    if (!beamBreakStatuses[0]) {
      if (this.getDirection()){
        ballCount--;
      } else {
        ballCount++;
      }
    }
    return ballCount;
  }

  public void expelBalls() {
    if (ballCount > 2) {
      this.toggleIntake(true);
    } while (this.getBallCount() > 2){}
    this.stopIntake();
  }
}  //Don't delete, this is for main method.

// Steps to activate the code:
// Incriment ball count (If statement)
// Reverse motors if ball count > 2 (if else statement)
// Set back to forward motors if the ball count is below 2 
// There are no while loops



























































































// I'm totally not hiding ;]