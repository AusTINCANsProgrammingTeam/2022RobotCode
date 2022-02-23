// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common.hardware;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;

/* Wrapper for ColorSensorV3 that uses a TCA9548A I2C multiplexer to
access up to 8 color sensors with the same device address */
public class ColorSensorMuxed {
  private ColorSensorV3 sensors;
  private I2C i2cMux;
  private final int tca9548Addr = 0x70;
  private ArrayList<Integer> i2cPorts;

  public ColorSensorMuxed(int... ports) {
    i2cMux = new I2C(Port.kOnboard, tca9548Addr);
    i2cPorts = new ArrayList<Integer>();
    for (int p : ports) {
      if (setI2cPort(p)) {
        i2cPorts.add(p);
        // Initialize each device, only need to keep last object
        sensors = new ColorSensorV3(Port.kOnboard);
      } else {
        DriverStation.reportError("Could not initialize color sensor on I2C port " + p, false);
      }
    }
  }

  // @returns true if successful
  private boolean setI2cPort(int port) {
    // Each bit in the TCA9548 control register corresponds to an I2C port to enable
    byte[] muxCtrlReg = {(byte) (1 << port)};
    byte[] muxCtrlRegRead = new byte[1];
    // Write to TCA9548 control register and readback and verify
    // (I2C class returns false for I2C transaction successes.)
    boolean ret = i2cMux.writeBulk(muxCtrlReg);
    ret |= i2cMux.readOnly(muxCtrlRegRead, muxCtrlRegRead.length);
    ret |= muxCtrlRegRead[0] != muxCtrlReg[0];
    return !ret;
  }

  public Color[] getColors() {
    Color[] colors = new Color[i2cPorts.size()];
    int i = 0;
    for (int p : i2cPorts) {
      if (setI2cPort(p)) {
        colors[i] = sensors.getColor();
      } else {
        DriverStation.reportError("Failed to get color from sensor on I2C port " + p, false);
        colors[i] = new Color(0, 0, 0);
      }
      i++;
    }
    return colors;
  }

  public int[] getProximities() {
    int[] proximities = new int[i2cPorts.size()];
    int i = 0;
    for (int p : i2cPorts) {
      if (setI2cPort(p)) {
        proximities[i] = sensors.getProximity();
      } else {
        DriverStation.reportError("Failed to get proximity from sensor on I2C port " + p, false);
        proximities[i] = 0;
      }
    }
    return proximities;
  }
}
