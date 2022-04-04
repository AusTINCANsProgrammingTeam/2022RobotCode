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
  private double[] lastProxReads;
  private double[] lastColorReads;
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
    lastProxReads = new double[i2cPorts.size()];
    lastColorReads = new double[i2cPorts.size()];
  }

  public boolean configureMeasurementRates(MeasurementRate rate) {

    ColorSensorMeasurementRate colorRate;
    ProximitySensorMeasurementRate proxRate;
    // Color measurement rate is dependant on its resolution.
    // it will run slower if the programmed rate is too fast for the
    // resolution bits.
    ColorSensorResolution colorRes;
    boolean ret = true;

    switch (rate) {
      case kRate40Hz:
        colorRes = ColorSensorResolution.kColorSensorRes16bit;
        colorRate = ColorSensorMeasurementRate.kColorRate25ms;
        proxRate = ProximitySensorMeasurementRate.kProxRate25ms;
        break;
      case kRate20Hz:
        colorRes = ColorSensorResolution.kColorSensorRes17bit;
        colorRate = ColorSensorMeasurementRate.kColorRate50ms;
        proxRate = ProximitySensorMeasurementRate.kProxRate50ms;
        break;
      case kRate5Hz:
        colorRes = ColorSensorResolution.kColorSensorRes19bit;
        colorRate = ColorSensorMeasurementRate.kColorRate200ms;
        proxRate = ProximitySensorMeasurementRate.kProxRate200ms;
        break;
      case kRate10Hz:
      default:
        colorRes = ColorSensorResolution.kColorSensorRes18bit;
        colorRate = ColorSensorMeasurementRate.kColorRate100ms;
        proxRate = ProximitySensorMeasurementRate.kProxRate100ms;
        break;
    }
    for (int p : i2cPorts) {
      if (setI2cPort(p)) {
        sensors.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, proxRate);
        sensors.configureColorSensor(colorRes, colorRate, GainFactor.kGain3x);
      } else {
        DriverStation.reportError(
            "Failed to configure sample rate of color sensor on I2C port " + p, false);
        ret = false;
      }
    }
    if (ret) {
      sensorPeriodInSeconds = rate.period;
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

  public Color getColor(int i2cPort) {
    int i = i2cPorts.indexOf(i2cPort);
    if (i == -1) {
      DriverStation.reportError("Invalid I2C port " + i2cPort, false);
      return new Color(0, 0, 0);
    }
    if (Timer.getFPGATimestamp() - lastColorReads[i] > sensorPeriodInSeconds) {
      if (setI2cPort(i2cPort)) {
        colors[i] = sensors.getColor();
      } else {
        DriverStation.reportError("Failed to get color from sensor on I2C port " + i2cPort, false);
        colors[i] = new Color(0, 0, 0);
      }
      lastColorReads[i] = Timer.getFPGATimestamp();
    }
    return colors[i];
  }

  public int getProximity(int i2cPort) {

    int i = i2cPorts.indexOf(i2cPort);
    if (i == -1) {
      DriverStation.reportError("Invalid I2C port " + i2cPort, false);
      return 0;
    }
    if (Timer.getFPGATimestamp() - lastProxReads[i] > sensorPeriodInSeconds) {
      if (setI2cPort(i2cPort)) {
        proximities[i] = sensors.getProximity();
      } else {
        DriverStation.reportError(
            "Failed to get proximity from sensor on I2C port " + i2cPort, false);
        proximities[i] = 0;
      }
    }
    lastProxReads[i] = Timer.getFPGATimestamp();
    return proximities[i];
  }

  public int[] getProximities() {
    int i = 0;
    for (int p : i2cPorts) {
      proximities[i] = getProximity(p);
      i++;
    }
    return proximities;
  }

  public Color[] getColors() {
    int i = 0;
    for (int p : i2cPorts) {
      colors[i] = getColor(p);
      i++;
    }
    return colors;
  }
}
