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
import java.util.Arrays;
import java.util.Random;

public class CDSSubsystem extends SubsystemBase {
  public enum ManagementState {
    IDLE,
    EJECT,
    SHOOTER_EJECT,
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

  private SimDeviceSim colorSenseSim;
  private SimDouble m_simR, m_simG, m_simB, m_simProx;

  private int simCount = 0;
  private String ballLayout;

  private int msCurrent = 0;
  private int idleEnterOffset = 2500;

  /*
  private int ejectRuntime = 2000; // amount of time auto eject will run intake backwards for in ms
  private int advanceTimeout = 2000; // how long CDS should run before it times out*/

  private int ballCount = 0;
  private double colorThreshold = 0.5; // TODO: change during testing

  private int[] sensorStatuses;
  private boolean[] activationArray = new boolean[3];
  private Color[] colors = new Color[3];
  private String lastFrontBallColor;
  private String lastMiddleBallColor;
  private String lastTopBallColor;


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
  private NetworkTableEntry frontBallColor = CDSTab.add("Front Ball Color", "None").getEntry();
  private NetworkTableEntry middleBallColor = CDSTab.add("Middle Ball Color", "None").getEntry();
  private NetworkTableEntry topBallColor = CDSTab.add("Top Ball Color", "None").getEntry();
  private NetworkTableEntry frontSenseBlue = CDSTab.add("Front Sense Blue",  0).getEntry();
  private NetworkTableEntry frontSenseRed = CDSTab.add("Front Sense Red", 0).getEntry();
  private NetworkTableEntry middleSenseBlue = CDSTab.add("Middle Sense Blue",  0).getEntry();
  private NetworkTableEntry middleSenseRed = CDSTab.add("Middle Sense Red", 0).getEntry();
  private NetworkTableEntry topSenseBlue = CDSTab.add("Top Sense Blue",  0).getEntry();
  private NetworkTableEntry topSenseRed = CDSTab.add("Top Sense Red", 0).getEntry();
  // private NetworkTableEntry CDSBallCount =
  // CDSTab.add("Ball Count", 0).getEntry();
  private NetworkTableEntry frontSensorProx = CDSTab.add("Front Proximity", 0).getEntry();
  private NetworkTableEntry middleSensorProx = CDSTab.add("Middle Proximity", 0).getEntry();
  private NetworkTableEntry backSensorProx = CDSTab.add("Back Proximity", 0).getEntry();
  private NetworkTableEntry CDSBallCount = CDSTab.add("Ball Count", 0).getEntry();
  private NetworkTableEntry CDSState = CDSTab.add("CDS State", "IDLE").getEntry();
  private NetworkTableEntry managementOnOff =
      CDSTab.add("Run Auto Intake and Eject", true)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withPosition(1, 2)
          .getEntry();
  private NetworkTableEntry ballLayoutEntry = CDSTab.add("CDS Ball Layout", "000").getEntry();

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
        SmartDashboard.putNumber("CDS Wheel Speed", -Constants.CDSWheelControllerSpeed);
      }

      CDSBeltController.set(-Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction", "Reverse");
        SmartDashboard.putNumber("CDS Belt Speed", -Constants.CDSBeltSpeed);
      }
    } else {
      DCDSSpeed.setDouble(1);
      CDSWheelControllerOne.set(Constants.CDSWheelControllerSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Wheel Direction", "Forward");
        SmartDashboard.putNumber("CDS Wheel Speed", Constants.CDSWheelControllerSpeed);
      }

      CDSBeltController.set(Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction", "Forward");
        SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
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
        SmartDashboard.putNumber("CDS Wheel Speed", Constants.CDSWheelControllerSpeed);
      }
    }
  }

  public void CDSBeltToggle(boolean reverse) {
    DCDSSpeed.setDouble(-1);
    if (reverse) {
      CDSBeltController.set(-Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction", "Reverse");
        SmartDashboard.putNumber("CDS Belt Speed", -Constants.CDSBeltSpeed);
      }
    } else {
      DCDSSpeed.setDouble(1);
      CDSBeltController.set(Constants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction", "Forward");
        SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
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
      SmartDashboard.putNumber("CDS Wheel Speed", 0.0);
      SmartDashboard.putNumber("CDS Belt Speed", 0.0);
    }
  }

  public void stopCDSWheel() {
    // Stops only the centering wheels
    CDSWheelControllerOne.set(0.0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("CDS Wheel Speed", 0.0);
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
    if (Robot.isSimulation()) {
      Random rand = new Random();

      for (int i = 0; i < 3; i++) {
        // fill in random proximity values that are either greater than or less than the sensor
        // threshold values
        activationArray[i] = rand.nextBoolean();
      }

      ballCount = 0;
      for (boolean status : activationArray) {
        if (status) {
          ballCount++;
        }
      }

      return activationArray;
    }

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

  public String[] senseColor() {
    if (currentColorCycle % cycleWait == 0) {
      currentColorCycle = 0;
      colors = colorSensors.getColors();

      double frontRedRatio = colors[2].red;
      double frontBlueRatio = colors[2].blue;
      double middleRedRatio = colors[1].red;
      double middleBlueRatio = colors[1].blue;
      double topRedRatio = colors[0].red;
      double topBlueRatio = colors[0].blue;
      frontSenseBlue.setNumber(frontBlueRatio);
      frontSenseRed.setNumber(frontRedRatio);
      middleSenseBlue.setNumber(middleBlueRatio);
      middleSenseRed.setNumber(middleRedRatio);
      topSenseBlue.setNumber(topBlueRatio);
      topSenseRed.setNumber(topRedRatio);

      // Only sensing colors for first sensor
      // dealing with any other complexities
      if (frontRedRatio > colorThreshold) {
        frontBallColor.setString("Red");
        lastFrontBallColor = "Red";
      } else if (frontBlueRatio > colorThreshold) {
        frontBallColor.setString("Blue");
        lastFrontBallColor = "Blue";
      } else {
        frontBallColor.setString("None");
        lastFrontBallColor = "None";
      }

      //sensing middle color
      if (middleRedRatio > colorThreshold) {
        middleBallColor.setString("Red");
        lastMiddleBallColor = "Red";
      } else if (middleBlueRatio > colorThreshold) {
        middleBallColor.setString("Blue");
        lastMiddleBallColor = "Blue";
      } else {
        middleBallColor.setString("None");
        lastMiddleBallColor = "None";
      }

      //sensing top color
      if (topRedRatio > colorThreshold) {
        topBallColor.setString("Red");
        lastTopBallColor = "Red";
      } else if (topBlueRatio > colorThreshold) {
        topBallColor.setString("Blue");
        lastTopBallColor = "Blue";
      } else {
        topBallColor.setString("None");
        lastTopBallColor = "None";
      }
    }

    currentColorCycle++;
    String[] lastSensorColors = {lastFrontBallColor, lastMiddleBallColor, lastTopBallColor};

    return lastSensorColors;
  }

  public String[] senseAllColors() {
    if (Robot.isSimulation()) {
      // on or off prox
      String[] simColors = new String[3];
      Random rand = new Random();

      String[] colorChoices = new String[] {"Red", "Blue"};
      for (int i = 0; i < 3; i++) {
        // randomly select color for each sensor
        simColors[i] = colorChoices[rand.nextInt(colorChoices.length)];
      }
      frontBallColor.setString(simColors[0]);
      middleBallColor.setString(simColors[1]);
      topBallColor.setString(simColors[2]);

      return simColors;
    }

    if (currentColorCycle % cycleWait == 0) {
      colors = colorSensors.getColors();
      currentColorCycle = 0;
    }

    String[] sensedColors = new String[3];

    for (int i = 0; i < 3; i++) {
      double magnitude = colors[i].red + colors[2].green + colors[2].blue;
      double redRatio = colors[i].red / magnitude;
      double blueRatio = colors[i].blue / magnitude;

      if (redRatio > colorThreshold) {
        sensedColors[i] = "Red";
      } else if (blueRatio > colorThreshold) {
        sensedColors[i] = "Blue";
      } else {
        sensedColors[i] = "None";
      }
    }
    frontBallColor.setString(sensedColors[0]);
    middleBallColor.setString(sensedColors[1]);
    topBallColor.setString(sensedColors[2]);

    return sensedColors;
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

  int count = 0;
  int runCount = 0;

  public void changeState() {
    getSensorStatus();
    ballCount = getBallCount();
    String[] sensedBallColors = senseColor();

    String[] ballLayoutArray = new String[] {"0", "0", "0"};
    for (int i = 0; i < 3; i++) {
      if (activationArray[i] && sensedBallColors[i] == allianceColor) {
        ballLayoutArray[i] = "2";
      } else if (activationArray[i] && sensedBallColors[i] != allianceColor) {
        ballLayoutArray[i] = "1";
      }
    }

    ballLayout = ballLayoutArray[0] + ballLayoutArray[1] + ballLayoutArray[2];

    // offset for entering idle so that we don't enter it while balls are in transit
    if (managementEnabled()) {
      if (Robot.isSimulation()) {
        // if robot is in simulation, ignore about time offsets
        msCurrent = idleEnterOffset + 1;
      }

      if (Arrays.asList(Constants.idleStates).contains(ballLayout)
          && msCurrent >= idleEnterOffset) {
        state = ManagementState.IDLE;
        msCurrent = 0;
      } else {
        msCurrent += 20;
      }

      if (Arrays.asList(Constants.advanceStates).contains(ballLayout)) {
        state = ManagementState.ADVANCE;
        msCurrent = 0;
      } else if (Arrays.asList(Constants.intakeEjectStates).contains(ballLayout)) {
        state = ManagementState.EJECT;
        msCurrent = 0;
      } else if (Arrays.asList(Constants.shooterEjectStates).contains(ballLayout)) {
        state = ManagementState.SHOOTER_EJECT;
        msCurrent = 0;
      }

      CDSState.setString(state.toString());
      ballLayoutEntry.setString(ballLayout);
    }
  }

  public void newColorSim() {
    if (Robot.isSimulation()) {
      if (simCount == 250) {
        SmartDashboard.putString("CDS Sim Ball Layout", ballLayout);
        SmartDashboard.putString("CDS Sim State", state.toString());

        simCount = 0;
      } else {
        simCount++;
      }
    }
  }

  public boolean managementEnabled() {
    return managementOnOff.getBoolean(true);
  }

  public void periodic() {}
}
