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
  private CANSparkMax m_motorController1 = new CANSparkMax(Constants.kMotorOneID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax m_motorController2 = new CANSparkMax(Constants.kMotorTwoID, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  public IntakeSubsystem(boolean reverse, boolean on){
    m_motorController2.follow(m_motorController1);

    if (on){
      m_motorController1.set(Constants.kMotorSpeed);

      if (reverse){
        m_motorController1.setInverted(true);
      }
      else {
        m_motorController1.setInverted(false);
      }
    }
    else{
      m_motorController1.set(0);
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