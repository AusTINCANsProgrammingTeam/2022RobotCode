// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LimelightSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private PIDController m_PidController;
  private boolean isFinished;

  public LimelightSubsystem() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(1);
    m_PidController = new PIDController(0.1, 0, 0);
    m_PidController.setTolerance(2.0);
    isFinished = false;
  }

  public double getTX() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(3);
    // Gets TX, the horizontal angle of the target from the limelight
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double calculatePID() {
    double calculation = MathUtil.clamp(m_PidController.calculate(getTX(), 0.0), -1.0, 1.0);
    // Uses TX and our setpoint (which will always be 0.0) to return the next calculation
    if (m_PidController
        .atSetpoint()) { // If our robot is aligned within the tolerance, return 0.0 to end command
      isFinished = true;
      return 0.0;
    } else { // Returns calculation, with a minimum power level of 0.07 going to motors
      return Math.signum(calculation) * Math.max(Math.abs(calculation), 0.07);
    }
  }

  public boolean getFinished() {
    return isFinished;
  }

  public void reset() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(1);
    isFinished = false;
  }

  public void updateSmartDashboard() {

  }
}
