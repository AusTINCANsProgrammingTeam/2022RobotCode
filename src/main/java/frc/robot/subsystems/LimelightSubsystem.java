// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.common.hardware.MotorController;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.hardware.MotorController;



/** Add your docs here. */
public class LimelightSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private PIDController m_PidController;
  private DriveBaseSubsystem m_DriveBaseSubsystem;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;

  public LimelightSubsystem() {
    m_PidController = new PIDController(6e-5, 0, 0);
    m_PidController.setTolerance(1.0);
    m_leftMotor = m_DriveBaseSubsystem.getLeftMotor();
    m_rightMotor = m_DriveBaseSubsystem.getRightMotor();
  }

  public double getTX() {
    // Gets TX, the horizontal angle of the target from the limelight
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double calculatePID(){
      // Uses TX and our setpoint (which will always be 0.0) to return the next calculation
    if (m_PidController.atSetpoint()){ // If our robot is aligned within the tolerance, return 0.0 to end command
        return 0.0;
    }
    else{
        return m_PidController.calculate(getTX(), 0.0); 
    }
  }

  public void setMotors(){
    //Sets drive motors to align based on our calculations
    m_leftMotor.set(calculatePID());
    m_rightMotor.set(calculatePID());
  }

  public void updateSmartDashboard(){

}
}