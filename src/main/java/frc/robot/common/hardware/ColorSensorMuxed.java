// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common.hardware;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import java.util.ArrayList;

/* Wrapper for ColorSensorV3 that uses a TCA9548A I2C multiplexer to
access up to 8 color sensors with the same device address */
public class ColorSensorMuxed {
  private ColorSensorV3 sensors;
  private I2C i2cMux;
  private final int tca9548Addr = 0x70;
  private ArrayList<Integer> i2cPorts;
  private double lastProxRead;
  private double lastColorRead;
  private int[] proximities;
  private Color[] colors;
  private double sensorPeriodInSeconds = MeasurementRate.kRate10Hz.period;

  public enum MeasurementRate {
    kRate40Hz(.025),
    kRate20Hz(.05),
    kRate10Hz(.1),
    kRate5Hz(.2);

    public final double period;

    MeasurementRate(double i) {
      period = i;
    }
  }

  public ColorSensorMuxed(int... ports) {
    i2cMux = new I2C(Port.kMXP, tca9548Addr);
    i2cPorts = new ArrayList<Integer>();
    for (int p : ports) {
      i2cPorts.add(p);
      if (setI2cPort(p)) {
        // Initialize each device, only need to keep last object
        if (Robot.isReal() || sensors == null) {
          sensors = new ColorSensorV3(Port.kMXP);
        }
      } else {
        DriverStation.reportError("Could not initialize color sensor on I2C port " + p, false);
      }
    }
    proximities = new int[i2cPorts.size()];
    colors = new Color[i2cPorts.size()];
    lastProxRead = 0;
    lastColorRead = 0;
  }

  public boolean configureMeasurementRates(MeasurementRate rate) {

    ColorSensorMeasurementRate colorRate;
    ProximitySensorMeasurementRate proxRate;
    sensorPeriodInSeconds = rate.period;
    boolean ret = true;

    switch (rate) {
      case kRate40Hz:
        colorRate = ColorSensorMeasurementRate.kColorRate25ms;
        proxRate = ProximitySensorMeasurementRate.kProxRate25ms;
        break;
      case kRate20Hz:
        colorRate = ColorSensorMeasurementRate.kColorRate50ms;
        proxRate = ProximitySensorMeasurementRate.kProxRate50ms;
        break;
      case kRate5Hz:
        colorRate = ColorSensorMeasurementRate.kColorRate200ms;
        proxRate = ProximitySensorMeasurementRate.kProxRate200ms;
        break;
      case kRate10Hz:
      default:
        colorRate = ColorSensorMeasurementRate.kColorRate100ms;
        proxRate = ProximitySensorMeasurementRate.kProxRate100ms;
        break;
    }
    for (int p : i2cPorts) {
      if (setI2cPort(p)) {
        sensors.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, proxRate);
        sensors.configureColorSensor(
            ColorSensorResolution.kColorSensorRes20bit, colorRate, GainFactor.kGain3x);
      } else {
        DriverStation.reportError(
            "Failed to configure sample rate of color sensor on I2C port " + p, false);
        ret = false;
      }
    }
    return ret;
  }

  // @returns true if successful
  private boolean setI2cPort(int port) {
    // Each bit in the TCA9548 control register corresponds to an I2C port to enable
    byte[] muxCtrlReg = {(byte) (1 << port)};
    byte[] muxCtrlRegRead = new byte[1];
    // Write to TCA9548 control register and readback and verify
    // (I2C class returns false for I2C transaction successes.)
    if (Robot.isReal()) {
      boolean ret = i2cMux.writeBulk(muxCtrlReg);
      ret |= i2cMux.readOnly(muxCtrlRegRead, muxCtrlRegRead.length);
      ret |= muxCtrlRegRead[0] != muxCtrlReg[0];
      return !ret;
    } else {
      return true;
    }
  }

  public Color[] getColors() {
    if (Timer.getFPGATimestamp() - lastColorRead > sensorPeriodInSeconds) {
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
      lastColorRead = Timer.getFPGATimestamp();
    }
    return colors;
  }

  public int[] getProximities() {
    if (Timer.getFPGATimestamp() - lastProxRead > sensorPeriodInSeconds) {
      int i = 0;
      for (int p : i2cPorts) {
        if (setI2cPort(p)) {
          proximities[i] = sensors.getProximity();
        } else {
          DriverStation.reportError("Failed to get proximity from sensor on I2C port " + p, false);
          proximities[i] = 0;
        }
        i++;
      }
      lastProxRead = Timer.getFPGATimestamp();
    }
    return proximities;
  }
}
