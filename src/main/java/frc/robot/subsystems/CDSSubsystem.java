// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.hardware.MotorController;

/** Add your docs here. */
public class CDSSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController mCDSBeltController = new MotorController("CDS Belt Controller", Constants.kCDSMotorThreeID);
  private MotorController mCDSWheelControllerOne = new MotorController("Main CDS Wheel Controller", Constants.kCDSMotorFourID);
  private MotorController mCDSWheelControllerTwo = new MotorController("Follows CDS Wheel Controller", Constants.kCDSMotorFiveID);

  //TODO: ADD FOLLOW

  public CDSSubsystem() {
    mCDSBeltController = new MotorController("CDS Motor", Constants.kCDSMotorThreeID);
  }

  public void HopperSwitch(boolean on) {
    if (on) {
      mCDSBeltController.getSparkMax().set(Constants.kCDSMotorSpeed);
      SmartDashboard.putNumber("CDS Motor Speed", Constants.kCDSMotorSpeed);
    } else {
      mCDSBeltController.getSparkMax().set(0.0);
      SmartDashboard.putNumber("CDS Motor Speed", 0);
    }
  }

  public void ForwardCDS() {
    mCDSBeltController.getSparkMax().setInverted(false);
    SmartDashboard.putString("CDS Motor Direction", "Forward");
  }

  public void ReverseCDS() {
    mCDSBeltController.getSparkMax().setInverted(true);
    SmartDashboard.putString("CDS Motor Direction", "Reverse");
  }

}