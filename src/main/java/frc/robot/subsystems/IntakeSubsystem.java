// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private MotorController intakeMotorControllerOne;

  public IntakeSubsystem() {
    intakeMotorControllerOne =
        new MotorController("Intake Motor One", Constants.intakeMotorOneID, 40);
    
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
  // TODO: Gutted code
  /*
  public boolean getDirection() {
    // true = inverted, false = forward
    if (intakeMotorControllerOne.getSparkMax().get() > 0) {
      return false;
    } else {
    return true;
    }
  }
  */

  /*public boolean[] getBeamBreakStatus() {
    boolean[] beamBreakArray;
    beamBreakArray = new boolean[3];
    beamBreakArray[0] = frontBallSensor.get();
    beamBreakArray[1] = middleBallSensor.get();
    beamBreakArray[2] = finalBallSensor.get();
    return beamBreakArray;
  }*/

  public void periodic() {
    /*boolean[] statusArray = getBeamBreakStatus();
    SmartDashboard.putBooleanArray("Beam Break", statusArray);*/
  }

  /*
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
    }
    this.stopIntake();
  }
  */

} // Don't delete, this is for main method.
