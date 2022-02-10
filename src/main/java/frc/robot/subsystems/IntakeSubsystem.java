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
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private MotorController intakeMotorControllerOne;
  private MotorController intakeMotorControllerTwo;
  //private MotorController CDSWheelControllerOne;
  //private MotorController CDSWheelControllerTwo;
  private int ballCount = 0;

  public IntakeSubsystem() {
    intakeMotorControllerOne = new MotorController("Intake Motor One", Constants.intakeMotorOneID, 40);
    //TODO: Gutted code
    //intakeMotorControllerTwo = new MotorController("Intake Motor Two", Constants.intakeMotorTwoID, 40); 
    //CDSWheelControllerOne = new MotorController("Wheel Motor Controller 1", Constants.intakeWheelOneID, 40);
    //CDSWheelControllerTwo = new MotorController("Wheel Motor Controller 2", Constants.intakeWheelTwoID, 40);
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
}  //Do not delete, this is for main method.
