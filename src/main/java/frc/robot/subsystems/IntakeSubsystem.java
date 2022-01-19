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
  
  private MotorController mIntakeMotorControllerOne;
  private MotorController mIntakeMotorControllerTwo;

  public IntakeSubsystem() {
    mIntakeMotorControllerOne = new MotorController("Intake Motor One", Constants.kIntakeMotorOneID);
    mIntakeMotorControllerTwo = new MotorController("Intake Motor Two", Constants.kIntakeMotorTwoID);

    mIntakeMotorControllerTwo.getSparkMax().follow(mIntakeMotorControllerTwo.getSparkMax());
  }

  public void IntakeSwitch(boolean on){    
    if (on){
      double intakeSmartSpeed = SmartDashboard.getNumber("Belt Speed", Constants.kCDSBeltSpeed);
      
      mIntakeMotorControllerOne.getSparkMax().set(intakeSmartSpeed);
      SmartDashboard.putNumber("Intake Motor Speed", Constants.kIntakeMotorSpeed);
    } else {
      mIntakeMotorControllerOne.getSparkMax().set(0);
      SmartDashboard.putNumber("Intake Motor Speed", 0);
    }
  }
  
  public void ForwardIntake(){
    mIntakeMotorControllerOne.getSparkMax().setInverted(false);
    SmartDashboard.putString("Intake Motor Direction", "Forward");
  }

  public void ReverseIntake(){
    mIntakeMotorControllerOne.getSparkMax().setInverted(true);
    SmartDashboard.putString("Intake Motor Direction", "Reverse");
  }

  
}
