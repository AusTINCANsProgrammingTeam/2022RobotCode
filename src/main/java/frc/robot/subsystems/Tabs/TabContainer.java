// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Tabs;

import frc.robot.subsystems.DriveBaseSubsystem;

/** Add your docs here. */
public class TabContainer {

    private TabDriveBase mTabDriveBase;

    public TabContainer(DriveBaseSubsystem d) {
        mTabDriveBase = new TabDriveBase(d);
    }

    public void periodic() {
        mTabDriveBase.periodic();
    }
}
