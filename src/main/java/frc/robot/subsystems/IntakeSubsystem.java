// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax mIntakeMotorController1;
  private CANSparkMax mIntakeMotorController2;
  
  public IntakeSubsystem() {
    mIntakeMotorController1 = new CANSparkMax(Constants.kIntakeMotorOneID, CANSparkMaxLowLevel.MotorType.kBrushless);
    mIntakeMotorController2 = new CANSparkMax(Constants.kIntakeMotorTwoID, CANSparkMaxLowLevel.MotorType.kBrushless);
    mIntakeMotorController2.follow(mIntakeMotorController1, false);
  }

  public void IntakeSwitch(boolean on){    
    if (on){
      mIntakeMotorController1.set(Constants.kIntakeMotorSpeed);
      SmartDashboard.putNumber("Intake Motor Speed", Constants.kIntakeMotorSpeed);
    } else {
      mIntakeMotorController1.set(0);
      SmartDashboard.putNumber("Intake Motor Speed", 0);
    }
  }
  

  public void ForwardIntake(){
    mIntakeMotorController1.setInverted(false);
    SmartDashboard.putString("Intake Motor Direction", "Forward");

  }

  public void ReverseIntake(){
    mIntakeMotorController1.setInverted(true);
    SmartDashboard.putString("Intake Motor Direction", "Reverse");
  }
}
