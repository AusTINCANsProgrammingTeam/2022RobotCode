// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Tabs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveBaseSubsystem;

/** Add your docs here. */
public class TabContainer extends SubsystemBase {
  private TabDriveBase mTabDriveBase;

  public TabContainer(DriveBaseSubsystem d) {
    if (d != null) {
      mTabDriveBase = new TabDriveBase(d);
    }
  }

  public void periodic() {
    mTabDriveBase.periodic();
  }
}
