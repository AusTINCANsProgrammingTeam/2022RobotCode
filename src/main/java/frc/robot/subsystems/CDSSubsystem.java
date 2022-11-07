// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CDSConstants;
import frc.robot.common.hardware.MotorController.MotorConfig;
import frc.robot.common.hardware.MotorController;

public class CDSSubsystem extends SubsystemBase {
  private CANSparkMax CDSBeltMotor;
  private CANSparkMax singulatorOneMotor;
  private CANSparkMax singulatorTwoMotor;

  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry CDSIndicator =
      operatorTab
          .add("CDS Indicator", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(3, 1)
          .getEntry();


  public CDSSubsystem() {
    singulatorOneMotor = MotorController.constructMotor(MotorConfig.singulatorOne);
    singulatorTwoMotor = MotorController.constructMotor(MotorConfig.singulatorTwo);
    CDSBeltMotor = MotorController.constructMotor(MotorConfig.CDSBelt);

    singulatorTwoMotor.follow(singulatorOneMotor, true);
    singulatorOneMotor.setIdleMode(IdleMode.kCoast);
    CDSBeltMotor.setIdleMode(IdleMode.kBrake);
  }

  public void runCDS(boolean reversed) {
    if (reversed) {
      CDSIndicator.setDouble(-1);
      singulatorOneMotor.set(-CDSConstants.singulatorSpeed);
      CDSBeltMotor.set(-CDSConstants.beltSpeed);
    } else {
      CDSIndicator.setDouble(1);
      singulatorOneMotor.set(CDSConstants.singulatorSpeed);
      CDSBeltMotor.set(CDSConstants.beltSpeed);
    }
  }

  public void runBelt(boolean reversed) {
    if (reversed) {
      CDSIndicator.setDouble(-1);
      CDSBeltMotor.set(-CDSConstants.beltSpeed);
    } else {
      CDSIndicator.setDouble(1);
      CDSBeltMotor.set(CDSConstants.beltSpeed);
    }
  }

  public void stopCDS() {
    CDSIndicator.setDouble(0);
    singulatorOneMotor.set(0.0);
    CDSBeltMotor.set(0.0);
  }

  public void stopBelt() {
    CDSBeltMotor.set(0.0);
  }
}