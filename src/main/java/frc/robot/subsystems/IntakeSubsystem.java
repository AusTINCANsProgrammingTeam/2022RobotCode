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
  private CANSparkMax m_intakeMotorController1 = new CANSparkMax(Constants.kIntakeMotorOneID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax m_intakeMotorController2 = new CANSparkMax(Constants.kIntakeMotorTwoID, CANSparkMaxLowLevel.MotorType.kBrushless);

  public void IntakeController(){

  }
  
  public void IntakeSwitch(boolean on){
    m_intakeMotorController2.follow(m_intakeMotorController1, false);

    if (on){
      m_intakeMotorController1.set(Constants.kIntakeMotorSpeed);
    } else {
      m_intakeMotorController1.set(0);
    }
  }

  public void ForwardIntake(){
    m_intakeMotorController1.setInverted(false);
    m_intakeMotorController2.setInverted(false);
  }

  public void ReverseIntake(){
    m_intakeMotorController1.setInverted(true);
    m_intakeMotorController2.setInverted(true);
  }
}