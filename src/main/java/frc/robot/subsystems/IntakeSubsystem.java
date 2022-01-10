// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;


/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax m_motorController1 = new CANSparkMax(Constants.kIntakeMotorOneID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax m_motorController2 = new CANSparkMax(Constants.kIntakeMotorTwoID, CANSparkMaxLowLevel.MotorType.kBrushless);
  

  public void IntakeController(){
  
  }
  
  public void IntakeSwitch(boolean on){
    if (on){
      m_motorController1.set(Constants.kIntakeMotorSpeed);
      m_motorController2.set(Constants.kIntakeMotorSpeed);
    } else {
      m_motorController1.set(0);
      m_motorController2.set(0);
    }
  }

  public void ReverseIntake(boolean reverse){
    if (reverse){
      m_motorController1.setInverted(true);
      m_motorController2.setInverted(true);
    } else {
      m_motorController1.setInverted(false);
      m_motorController2.setInverted(false);
    } 

    
  }
  /*
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    
  }
  */
}