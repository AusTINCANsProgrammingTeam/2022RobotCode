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
  private int nextOpenSensor = -1;

  private SimDeviceSim colorSenseSim;
  private SimDouble m_simR, m_simG, m_simB, m_simProx;
  private int simCount = 0;

  private int msCurrent = 0;
  private int ejectRuntime = 650; // amount of time auto eject will run intake backwards for in ms
  private int advanceTimeout = 2000; // how long CDS should run before it times out

  private int ballCount = 0;
  private Color[] colors = new Color[3];

  private int[] sensorStatuses;
  private boolean[] activationArray = new boolean[3];
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
          .add("Run Auto Intake and Eject", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withPosition(5, 1)
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
    state = ManagementState.IDLE;
    if (Robot.isSimulation()) {
      colorSenseSim = new SimDeviceSim("REV Color Sensor V3", I2C.Port.kMXP.value, 82);
      m_simR = colorSenseSim.getDouble("Red");
      m_simG = colorSenseSim.getDouble("Blue");
      m_simB = colorSenseSim.getDouble("Green");
      m_simProx = colorSenseSim.getDouble("Proximity");
    }
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

  public void CDSBeltToggle(boolean reverse) {
    DCDSSpeed.setDouble(-1);
    if (reverse) {
      CDSBeltController.set(-Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction 5", "Reverse");
        SmartDashboard.putNumber("CDS Belt Speed 4", -Constants.CDSBeltSpeed);
      }
    } else {
      DCDSSpeed.setDouble(1);
      CDSBeltController.set(Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction 6", "Forward");
        SmartDashboard.putNumber("CDS Belt Speed 5", Constants.CDSBeltSpeed);
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

  public double getBeltSpeed() {
    return CDSBeltController.getSpeed();
  }

  public double getWheelSpeed() {
    return CDSWheelControllerOne.getSpeed();
  }

  public void stopCDS() {
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

  public boolean[] getSensorStatus() {
    if (currentProxCycle % cycleWait == 0) {
      currentProxCycle = 0;
      sensorStatuses = colorSensors.getProximities();
      // if (Constants.DebugMode) {
      frontSensorProx.setNumber(sensorStatuses[2]);
      middleSensorProx.setNumber(sensorStatuses[1]);
      backSensorProx.setNumber(sensorStatuses[0]);
      // }

      activationArray[0] = sensorStatuses[0] > Constants.backSensorActivation;
      activationArray[1] = sensorStatuses[1] > Constants.middleSensorActivation;
      activationArray[2] = sensorStatuses[2] > Constants.frontSensorActivation;

      ballCount = 0;
      for (boolean status : activationArray) {
        if (status) {
          ballCount++;
        }
      }
      if (Constants.DebugMode) {
        CDSBallCount.setNumber(ballCount);
      }
    }
    currentProxCycle++;
    return activationArray;
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

  public String senseColor() {
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

  public boolean sensorsOnline() {
    boolean isOnline = true;
    sensorsDown = 0;

    sensorStatuses = colorSensors.getProximities();
    for (int prox : sensorStatuses) {
      if (prox == 0) {
        sensorsDown += 1;
        isOnline = false;
      }
    }
    return isOnline;
  }

  public int getSensorDown() {
    return sensorsDown;
  }

  public String getAllianceColor() {
    return allianceColor;
  }

  public int getBallCount() {
    return ballCount;
  }

  public ManagementState getState() {
    return state;
  }

  public boolean managementEnabled() {
    return managementOnOff.getBoolean(false);
  }

  int count = 0;
  int runCount = 0;

  public void changeState() {
    getSensorStatus();
    ballCount = getBallCount();
    String sensedBallColor = senseColor();
    int currentOpenSensor = getNextOpenSensor();

    boolean ballPresent =
        activationArray[2]; // whether or not there's a ball at the centering wheels

    if (managementEnabled()) {
      switch (state) {
        case IDLE:
          nextOpenSensor = -1;
          msCurrent = 0;

          if ((ballCount > 2 || sensedBallColor != allianceColor) && ballPresent) {
            state = ManagementState.EJECT;
          }

          if ((ballCount < 3 && currentOpenSensor != -1 && ballPresent)) {
            state = ManagementState.ADVANCE;
            nextOpenSensor = currentOpenSensor;
          }

          break;
        case ADVANCE:
          if (sensedBallColor != allianceColor && ballPresent) {
            state = ManagementState.EJECT;
            msCurrent = 0;
          } else if (activationArray[nextOpenSensor] || msCurrent >= advanceTimeout) {
            state = ManagementState.IDLE;
          } else {
            msCurrent += 20;
          }

          break;
        case EJECT:
          if (msCurrent >= ejectRuntime) {
            state = ManagementState.IDLE;
          } else {
            msCurrent += 20;
          }

          break;
      }
    } else {
      state = ManagementState.IDLE;
    }
    CDSState.setString(state.toString());
  }

  public void simulateColorSense() {
    if (Robot.isSimulation()) {
      if (simCount == 500) {
        m_simProx.set(Constants.frontSensorActivation + 1);
        if (allianceColor == "Blue") {
          m_simB.set(.9);
        } else {
          m_simR.set(.9);
        }
      } else if (simCount == 1000) {
        m_simProx.set(50);
        m_simB.set(0);
        m_simR.set(0);

        simCount = 0;
      }
      simCount++;
    }
  }
}
