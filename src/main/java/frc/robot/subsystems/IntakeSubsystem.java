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
    intakeMotorControllerOne = new MotorController("Intake Motor One", Constants.intakeMotorOneID, 40);
    intakeMotorControllerOne.setInverted(true);
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

  public void periodic() {}

} // Do delete, not for main method