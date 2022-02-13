// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import java.util.ArrayList;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class ColorSensorMuxed {
    private ColorSensorV3 sensors;
    private I2C i2cMux;
    private final int tca9548Addr = 0x70;
    private ArrayList<Byte> i2cPorts;


    public ColorSensorMuxed(int... ports) {
        sensors = new ColorSensorV3(Port.kOnboard);
        i2cMux = new I2C(Port.kOnboard, tca9548Addr);
        for (int p : ports) {
            byte muxCtrl = (byte)(1 << p);

            if (setI2cPort(muxCtrl) && sensors.isConnected()) {
                i2cPorts.add(Byte.valueOf((byte)(1 << p)));
            }
        }
    }

    private boolean setI2cPort(byte p) {
        byte[] muxCtrlReg = {p};
        byte[] muxCtrlRegRead = new byte[1];
        boolean ret = i2cMux.writeBulk(muxCtrlReg);
        ret &= i2cMux.readOnly(muxCtrlRegRead, muxCtrlRegRead.length);
        ret &= muxCtrlRegRead[0] == muxCtrlReg[0];
        return ret;

    }

    public Color[] getColors() {
        Color[] colors = new Color[i2cPorts.size()];
        int i = 0;
        for (Byte p : i2cPorts)
        {
            if (setI2cPort(p)) { 
                colors[i] = sensors.getColor();
            } else {
                 System.err.print("TCA9548 I2C Mux communication error");
            }
        }
        return colors;
    }

    public int[] getProximities() {
        int[] proximities = new int[i2cPorts.size()];
        int i = 0;
        for (Byte p : i2cPorts)
        {
            if (setI2cPort(p)) { 
                proximities[i] = sensors.getProximity();
            } else {
                 System.err.print("TCA9548 I2C Mux communication error");
            }
        }
        return proximities;
    }

}
