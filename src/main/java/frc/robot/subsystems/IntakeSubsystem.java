// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private MotorController intakeMotorControllerOne;
  private MotorController intakeMotorControllerTwo;
  private MotorController CDSWheelControllerOne;
  private MotorController CDSWheelControllerTwo;

  public IntakeSubsystem() {
    intakeMotorControllerOne = new MotorController("Intake Motor One", Constants.intakeMotorOneID);
    CDSWheelControllerOne = new MotorController("Wheel Motor Controller 1", Constants.intakeWheelOneID, 40);
    CDSWheelControllerTwo = new MotorController("Wheel Motor Controller 2", Constants.intakeWheelTwoID, 40);

    // Remove invert=true parameter if wheels aren't running correctly
    CDSWheelControllerOne.getSparkMax().follow(intakeMotorControllerOne.getSparkMax());
    CDSWheelControllerTwo.getSparkMax().follow(intakeMotorControllerOne.getSparkMax(), true);
  }

  public void IntakeSwitch(boolean on){    
    if (on){
      //double intakeSmartSpeed = SmartDashboard.getNumber("Belt Speed", Constants.CDSBeltSpeed);
      
      intakeMotorControllerOne.getSparkMax().set(Constants.intakeMotorSpeed);
      SmartDashboard.putNumber("Intake Motor Speed", Constants.intakeMotorSpeed);
    } else {
      intakeMotorControllerOne.getSparkMax().set(0);
      SmartDashboard.putNumber("Intake Motor Speed", 0);
    }
  }
  
  public void ForwardIntake(){
    intakeMotorControllerOne.getSparkMax().set(Constants.intakeMotorSpeed);
    SmartDashboard.putString("Intake Motor Direction", "Forward");
  }

  public void ReverseIntake(){
    intakeMotorControllerOne.getSparkMax().set(-Constants.intakeMotorSpeed);
    SmartDashboard.putString("Intake Motor Direction", "Reverse");
  }

  
}
