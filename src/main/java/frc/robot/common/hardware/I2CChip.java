// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common.hardware;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.I2C.Port;

/** Add your docs here. */
public class I2CChip {
    I2C i2cInterfacer;
    
    public I2CChip() {
        Port colorSensorPort = Port.kOnboard;
        byte address = 0x70;
        i2cInterfacer = new I2C(colorSensorPort, address);
    }

    /*public void writeSomething() {
        i2cInterfacer.writeBulk();
    }*/
}
