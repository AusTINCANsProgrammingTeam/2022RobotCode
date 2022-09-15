// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.common.hardware.ColorSensorMuxed;
import frc.robot.common.hardware.ColorSensorMuxed.MeasurementRate;
import frc.robot.common.hardware.MotorController;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class CDSSubsystem extends SubsystemBase {
  public enum ManagementState {
    IDLE,
    EJECT,
    ADVANCE
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController CDSBeltController;
  private MotorController CDSWheelControllerOne;
  private MotorController CDSWheelControllerTwo;
  private String allianceColor;
  private ColorSensorMuxed colorSensors;
  private ManagementState state;

  private boolean isRunning = false;
  private int currentSensor = -1;
  private int msCurrent = 0;
  private int advanceTimeout = 2500;

  private int ballCount = 0;
  private Color[] colors = new Color[3];

  private boolean[] activationArray;
  private int[] sensorStatuses;
  private String lastBallColor;

  private int currentProxCycle = 0;
  private int currentColorCycle = 0;
  private int cycleWait = 1; // 1 means to read every cycle

  private int sensorsDown = 0;
  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry DCDSSpeed =
      operatorTab
          .add("CDS Speed", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(3, 1)
          .getEntry();

  private ShuffleboardTab CDSTab = Shuffleboard.getTab("CDS Tab");
  private NetworkTableEntry ballColor = CDSTab.add("Ball Color", "Blue").getEntry();
  // private NetworkTableEntry CDSBallCount =
  // CDSTab.add("Ball Count", 0).getEntry();
  private NetworkTableEntry frontSensorProx = CDSTab.add("Front Proximity", 0).getEntry();
  private NetworkTableEntry middleSensorProx = CDSTab.add("Middle Proximity", 0).getEntry();
  private NetworkTableEntry backSensorProx = CDSTab.add("Back Proximity", 0).getEntry();
  private NetworkTableEntry CDSBallCount = CDSTab.add("Ball Count", 0).getEntry();
  private NetworkTableEntry CDSState = CDSTab.add("CDS State", "IDLE").getEntry();
  private NetworkTableEntry managementOnOff =
      operatorTab
          .add("Auto CDS", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withPosition(3, 2)
          .getEntry();

  public CDSSubsystem() {
    // BManualCDS.setBoolean(Constants.); TODO: setup when manual cds toggle is merged
    CDSBeltController = new MotorController("CDS Motor", Constants.CDSBeltID);
    CDSBeltController.setInverted(true);
    CDSWheelControllerOne =
        new MotorController("Wheel Motor Controller 1", Constants.CDSWheelControllerOneID);
    CDSWheelControllerTwo =
        new MotorController("Wheel Motor Controller 2", Constants.CDSWheelControllerTwoID);

    CDSWheelControllerOne.setInverted(true);
    CDSWheelControllerTwo.follow(CDSWheelControllerOne, true);

    CDSBeltController.setIdleMode(IdleMode.kBrake);
    CDSWheelControllerOne.setIdleMode(IdleMode.kCoast);

    colorSensors = new ColorSensorMuxed(1, 2, 0); // front to back color sensor ports on new robotn
    colorSensors.configureMeasurementRates(MeasurementRate.kRate40Hz);
    sensorStatuses = colorSensors.getProximities();
    allianceColor = DriverStation.getAlliance().toString();
    SmartDashboard.putString("Alliance Color", allianceColor);
  }

  public void CDSToggleAll(boolean reverse) {
    if (reverse) {
      CDSWheelControllerOne.set(-Constants.CDSWheelControllerSpeed);
      DCDSSpeed.setDouble(-1);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Wheel Direction", "Reverse");
        SmartDashboard.putNumber("CDS Wheel Speed 2", -Constants.CDSWheelControllerSpeed);
      }

      CDSBeltController.set(-Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction 3", "Reverse");
        SmartDashboard.putNumber("CDS Belt Speed 2", -Constants.CDSBeltSpeed);
      }
    } else {
      DCDSSpeed.setDouble(1);
      CDSWheelControllerOne.set(Constants.CDSWheelControllerSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Wheel Direction", "Forward");
        SmartDashboard.putNumber("CDS Wheel Speed 3", Constants.CDSWheelControllerSpeed);
      }

      CDSBeltController.set(Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction 4", "Forward");
        SmartDashboard.putNumber("CDS Belt Speed 3", Constants.CDSBeltSpeed);
      }
    }
  }

  public void CDSWheelToggle(boolean reverse) {
    if (reverse) {
      CDSWheelControllerOne.set(-Constants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Direction", "Reverse");
    } else {
      CDSWheelControllerOne.set(Constants.CDSWheelControllerSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Wheel Direction", "Forward");
        SmartDashboard.putNumber("CDS Wheel Speed 4", Constants.CDSWheelControllerSpeed);
      }
    }
  }

  public void CDSBeltToggle(boolean reverse, double beltSpeed) {
    DCDSSpeed.setDouble(-1);
    if (reverse) {
      CDSBeltController.set(-beltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction 5", "Reverse");
        SmartDashboard.putNumber("CDS Belt Speed 4", -beltSpeed);
      }
    } else {
      DCDSSpeed.setDouble(1);
      CDSBeltController.set(beltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction 6", "Forward");
        SmartDashboard.putNumber("CDS Belt Speed 5", beltSpeed);
      }
    }
  }

  public void stopCDS() {
    isRunning = false;
    currentSensor = -1;
    msCurrent = 0;

    DCDSSpeed.setDouble(0);
    // stops all motors in the CDS
    CDSWheelControllerOne.set(0.0);
    CDSBeltController.set(0.0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("CDS Wheel Speed 5", 0.0);
      SmartDashboard.putNumber("CDS Belt Speed 6", 0.0);
    }
  }

  public void stopCDSWheel() {
    // Stops only the centering wheels
    CDSWheelControllerOne.set(0.0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("CDS Wheel Speed 6", 0.0);
    }
  }

  public void stopCDSBelt() {
    // Stops only the belt
    CDSBeltController.set(0.0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("CDS Belt Speed", 0.0);
    }
  }

  public boolean isBallPresent() {
    if (currentProxCycle % cycleWait == 0) {
      currentProxCycle = 0;
      sensorStatuses = colorSensors.getProximities();

      activationArray[0] = sensorStatuses[0] > Constants.backSensorActivation;
      activationArray[1] = sensorStatuses[1] > Constants.middleSensorActivation;
      activationArray[2] = sensorStatuses[2] > Constants.frontSensorActivation;
    }

    currentProxCycle++;
    return activationArray[2];
  }

  public int getNextOpenSensor() {
    // Starts at 0 and ends short of the centering wheel
    // length - 1 because the last index of the array is the first sensor, which isn't a valid point
    for (int i = 0; i < activationArray.length - 1; i++) {
      if (!activationArray[i]) {
        return i;
      }
    }
    return -1;
  }

  public String getBallColor() {
    if (currentColorCycle % cycleWait == 0) {
      currentColorCycle = 0;
      colors = colorSensors.getColors();
      SmartDashboard.putNumber("Front Sense B", colors[2].blue);
      SmartDashboard.putNumber("Front Sense R", colors[2].red);

      // Only sensing colors for first sensor so that we can handle it when it's coming in and not
      // dealing with any other complexities
      double redAmount = colors[2].red;
      double blueAmount = colors[2].blue;
      if (redAmount > blueAmount) {
        ballColor.setString("Red");
        lastBallColor = "Red";
      } else {
        ballColor.setString("Blue");
        lastBallColor = "Blue";
      }
    }

    currentColorCycle++;
    return lastBallColor;
  }

  public String getAllianceColor() {
    return allianceColor;
  }

  public int getBallCount() {
    return ballCount;
  }

  public boolean ballColorMatch() {
    return allianceColor == lastBallColor;
  }

  public boolean shouldAdvance() {
    // run if there's a ball at the sensor or there's a ball in transit
    return ballColorMatch() && isBallPresent() && ballCount <= 2;
  }

  public boolean ballAtTarget() {
    return activationArray[currentSensor];
  }

  public boolean ballTimeout() {
    msCurrent += 20;
    return msCurrent > advanceTimeout;
  }

  public Command runIntakeCommand() {
    currentSensor = getNextOpenSensor();
    isRunning = true;

    return new StartEndCommand(() -> CDSToggleAll(false), this::stopCDS, this);
  }

  public Command runAutoAdvanceCommand() {
    return new WaitUntilCommand(new Trigger(this::shouldAdvance))
    .andThen(runIntakeCommand())
    .withInterrupt(new Trigger(this::ballAtTarget))
    .withInterrupt(new Trigger(this::ballTimeout));
  }
}