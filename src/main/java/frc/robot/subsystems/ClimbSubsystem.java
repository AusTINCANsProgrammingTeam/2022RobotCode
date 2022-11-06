// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import frc.robot.common.hardware.MotorController.MotorConfig;

public class ClimbSubsystem extends SubsystemBase {

  // 1 = Right Side, 2 = Left Side
  private CANSparkMax armOneMotor;
  private CANSparkMax armTwoMotor;
  private CANSparkMax poleOneMotor;
  private CANSparkMax poleTwoMotor;

  private SparkMaxPIDController armOnePIDController;
  private SparkMaxPIDController armTwoPIDController;
  private SparkMaxPIDController poleOnePIDController;
  private SparkMaxPIDController poleTwoPIDController;


  private Servo servoOne;
  private Servo servoTwo;

  private boolean climbEnabled, hookLocked;

  private double armHeightOne;
  private double armHeightTwo;
  private double poleHeightOne;
  private double poleHeightTwo;
  private double armEncoderHeightOne;
  private double armEncoderHeightTwo;
  private double poleEncoderHeightOne;
  private double poleEncoderHeightTwo;

  // 1 = Right Side, 2 = Left Side
  private ShuffleboardTab climbTab;

  // Arm 1
  private NetworkTableEntry sbArmSpeedOne;
  private NetworkTableEntry sbArmTargettedOne;
  private NetworkTableEntry sbarmHeightOne;

  // Arm 2
  private NetworkTableEntry sbArmSpeedTwo;
  private NetworkTableEntry sbArmTargettedTwo;
  private NetworkTableEntry sbarmHeightTwo;

  // Pole 1
  private NetworkTableEntry sbpoleHeightOne;
  private NetworkTableEntry sbPoleSpeedOne;
  private NetworkTableEntry sbPoleTargettedOne;

  // Pole 2
  private NetworkTableEntry sbpoleHeightTwo;
  private NetworkTableEntry sbPoleSpeedTwo;
  private NetworkTableEntry sbPoleTargettedTwo;

  // hook Servos
  private ShuffleboardTab hookServos;
  private NetworkTableEntry servo1;
  private NetworkTableEntry servo2;

  // Other
  private NetworkTableEntry sbClimbEnable; // Displays ClimbEnable Boolean

  // Operator Tab
  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry DClimbHeight1 =
      operatorTab
          .add("Arm Height 1", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(6, 0)
          .getEntry();
  private NetworkTableEntry DClimbHeight2 =
      operatorTab
          .add("Arm Height 2", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(6, 1)
          .getEntry();
  private NetworkTableEntry DClimbHeight3 =
      operatorTab
          .add("Pole Height 1", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(8, 0)
          .getEntry();
  private NetworkTableEntry DClimbHeight4 =
      operatorTab
          .add("Pole Height 2", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(8, 1)
          .getEntry();
  private NetworkTableEntry BClimbEnabled =
      operatorTab
          .add("Climb Enabled", false)
          .withPosition(5, 0)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry();
  private NetworkTableEntry BAutomaticControl =
      operatorTab
          .add("Auto Climb Active", false)
          .withPosition(5, 1)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry();

  public ClimbSubsystem() {
    climbEnabled = false;

    // Shuffle Board Widgets
    climbTab = Shuffleboard.getTab("ClimbBase");

    // Arm 1
    sbArmTargettedOne =
        climbTab.add("Arm1 targetted", 0).withSize(3, 1).withPosition(2, 3).getEntry();
    sbArmSpeedOne = climbTab.add("Arm1 Speed", 0).withSize(2, 1).withPosition(5, 3).getEntry();

    // Arm 2
    sbArmTargettedTwo =
        climbTab.add("Arm2 targetted", 0).withSize(3, 1).withPosition(2, 4).getEntry();
    sbArmSpeedTwo = climbTab.add("Arm2 Speed", 0).withSize(2, 1).withPosition(5, 4).getEntry();

    // Pole 1
    sbPoleTargettedOne =
        climbTab.add("Pole1 targetted", 0).withSize(3, 1).withPosition(2, 0).getEntry();
    sbPoleSpeedOne = climbTab.add("Pole1 Speed", 0).withSize(2, 1).withPosition(5, 0).getEntry();

    // Pole 2
    sbPoleTargettedTwo =
        climbTab.add("Pole2 targetted", 0).withSize(3, 1).withPosition(2, 1).getEntry();
    sbPoleSpeedTwo = climbTab.add("Pole2 Speed", 0).withSize(2, 1).withPosition(5, 1).getEntry();

    hookServos = Shuffleboard.getTab("Hook Servos");
    servo1 = hookServos.add("Servo 1", 0.35).withSize(2, 2).withPosition(0, 0).getEntry();
    servo2 = hookServos.add("Servo 2", 1).withSize(2, 2).withPosition(2, 0).getEntry();

    // Arm 1 MotorController
    armOneMotor = MotorController.constructMotor(MotorConfig.climbArmOne);
    armOnePIDController = MotorController.constructPIDController(armOneMotor, Constants.armPosPID);
    armOneMotor.getEncoder().setPosition(0);
    armOneMotor.setIdleMode(IdleMode.kBrake);

    // Arm 2 MotorController
    armTwoMotor = MotorController.constructMotor(MotorConfig.climbArmTwo);
    armTwoPIDController = MotorController.constructPIDController(armTwoMotor, Constants.armPosPID);
    armTwoMotor.getEncoder().setPosition(0);
    armTwoMotor.setIdleMode(IdleMode.kBrake);

    // Pole 1 MotorController
    poleOneMotor = MotorController.constructMotor(MotorConfig.climbPoleOne);
    poleOnePIDController = MotorController.constructPIDController(poleOneMotor, Constants.polePosPID);
    poleOneMotor.getEncoder().setPosition(0);
    poleOneMotor.setIdleMode(IdleMode.kBrake);

    // Pole 2 MotorController
    poleTwoMotor = MotorController.constructMotor(MotorConfig.climbPoleTwo);
    poleTwoPIDController = MotorController.constructPIDController(poleTwoMotor, Constants.polePosPID);
    poleTwoMotor.getEncoder().setPosition(0);
    poleTwoMotor.setIdleMode(IdleMode.kBrake);

    // set the max output on each pid controller
    // range is plus minus the max output
    armOnePIDController.setOutputRange(-Constants.climbMaxOutput, Constants.climbMaxOutput);
    armTwoPIDController.setOutputRange(-Constants.climbMaxOutput, Constants.climbMaxOutput);
    poleOnePIDController.setOutputRange(-Constants.climbMaxOutput, Constants.climbMaxOutput);
    poleTwoPIDController.setOutputRange(-Constants.climbMaxOutput, Constants.climbMaxOutput);

    // set pid values
    armOnePIDController.setP(Constants.armVelocityPID[0], Constants.armVelPIDSlot);
    armOnePIDController.setI(Constants.armVelocityPID[1], Constants.armVelPIDSlot);
    armOnePIDController.setD(Constants.armVelocityPID[2], Constants.armVelPIDSlot);

    armTwoPIDController.setP(Constants.armVelocityPID[0], Constants.armVelPIDSlot);
    armTwoPIDController.setI(Constants.armVelocityPID[1], Constants.armVelPIDSlot);
    armTwoPIDController.setD(Constants.armVelocityPID[2], Constants.armVelPIDSlot);

    resetClimbHeights();

    // servos
    servoOne = new Servo(Constants.climbServoIDOne);
    servoTwo = new Servo(Constants.climbServoIDTwo);

    lockHooks();
  }

  public void resetClimbHeights() {
    armHeightOne = armOneMotor.getEncoder().getPosition();
    armHeightTwo = armTwoMotor.getEncoder().getPosition();

    poleHeightOne = poleOneMotor.getEncoder().getPosition();
    poleHeightTwo = poleTwoMotor.getEncoder().getPosition();
  }

  public void climbKeepDownFunction() {
    armOnePIDController.setReference(armHeightOne, CANSparkMax.ControlType.kPosition);
    armOnePIDController.setIMaxAccum(Constants.armSetIMaxAccum, 0);

    armTwoPIDController.setReference(armHeightTwo, CANSparkMax.ControlType.kPosition);
    armTwoPIDController.setIMaxAccum(Constants.armSetIMaxAccum, 0);

    poleOnePIDController.setReference(poleHeightOne, CANSparkMax.ControlType.kPosition);
    poleOnePIDController.setIMaxAccum(Constants.poleSetIMaxAccum, 0);

    poleTwoPIDController.setReference(poleHeightTwo, CANSparkMax.ControlType.kPosition);
    poleTwoPIDController.setIMaxAccum(Constants.poleSetIMaxAccum, 0);
  }

  public void toggleEnabled() {
    climbEnabled = !climbEnabled;
    if (climbEnabled) {
      armOneMotor.setSmartCurrentLimit(Constants.climbArmHighCurrent);
      armTwoMotor.setSmartCurrentLimit(Constants.climbArmHighCurrent);
      poleOneMotor.setSmartCurrentLimit(Constants.climbPoleHighCurrent);
      poleTwoMotor.setSmartCurrentLimit(Constants.climbPoleHighCurrent);
    } else {
      armOneMotor.setSmartCurrentLimit(Constants.climbArmLowCurrent);
      armTwoMotor.setSmartCurrentLimit(Constants.climbArmLowCurrent);
      poleOneMotor.setSmartCurrentLimit(Constants.climbPoleLowCurrent);
      poleTwoMotor.setSmartCurrentLimit(Constants.climbPoleLowCurrent);
    }
  }

  public void midClimb(double axisValue) {
    if (climbEnabled) {
        if (axisValue > 0) {
          if (armHeightOne + (axisValue * Constants.armUpSpeed) >= Constants.armHeightMin) {
            armHeightOne = armHeightOne + (axisValue * Constants.armUpSpeed);
          }
          if (armHeightTwo + (axisValue * Constants.armUpSpeed) >= Constants.armHeightMin) {
            armHeightTwo = armHeightTwo + (axisValue * Constants.armUpSpeed);
          }
        }
        if (axisValue < 0) {
          if (armHeightOne + (axisValue * Constants.armDownSpeed) <= Constants.armHeightMax) {
            armHeightOne = armHeightOne + (axisValue * Constants.armDownSpeed);
          }
          if (armHeightTwo + (axisValue * Constants.armDownSpeed) <= Constants.armHeightMax) {
            armHeightTwo = armHeightTwo + (axisValue * Constants.armDownSpeed);
          }
        }
      }
      armOnePIDController.setReference(armHeightOne, CANSparkMax.ControlType.kPosition);
      armTwoPIDController.setReference(armHeightTwo, CANSparkMax.ControlType.kPosition);
    }

  public void highArms(double axisValue) {
      if (axisValue > 0) {
        if (poleHeightOne + (axisValue * Constants.poleInSpeed) <= Constants.poleHeightMax) {
          poleHeightOne = poleHeightOne + (axisValue * Constants.poleInSpeed);
        }
        if (poleHeightTwo + (axisValue * Constants.poleInSpeed) <= Constants.poleHeightMax) {
          poleHeightTwo = poleHeightTwo + (axisValue * Constants.poleInSpeed);
        }
      }
      if (axisValue < 0) {
        if (poleHeightOne + (axisValue * Constants.poleOutSpeed)
            >= Constants.poleHeightMin) {
          poleHeightOne = poleHeightOne + (axisValue * Constants.poleOutSpeed);
        }
        if (poleHeightTwo + (axisValue * Constants.poleOutSpeed)
            >= Constants.poleHeightMin) {
          poleHeightTwo = poleHeightTwo + (axisValue * Constants.poleOutSpeed);
        }
      }
    poleOnePIDController.setReference(poleHeightOne, CANSparkMax.ControlType.kPosition);
    poleTwoPIDController.setReference(poleHeightOne, CANSparkMax.ControlType.kPosition);
  }

  // called at the start of auton
  public void retractPoles() {
    poleOnePIDController.setReference(Constants.poleHeightMax, CANSparkMax.ControlType.kPosition);
    poleTwoPIDController.setReference(Constants.poleHeightMax, CANSparkMax.ControlType.kPosition);
  }

  // functions for ClimbSequence1
  public void deployArms() {
    if (armEncoderHeightOne > Constants.armHeightFeather1
        || armEncoderHeightTwo > Constants.armHeightFeather1) {
      armOnePIDController
          .setReference(
              Constants.armFeatherRPM1, CANSparkMax.ControlType.kVelocity, Constants.armVelPIDSlot);
      armTwoPIDController
          .setReference(
              Constants.armFeatherRPM1, CANSparkMax.ControlType.kVelocity, Constants.armVelPIDSlot);
    } else if (armEncoderHeightOne > Constants.armHeightFeather2
        || armEncoderHeightTwo > Constants.armHeightFeather2) {
      armOnePIDController
          .setReference(
              Constants.armFeatherRPM2, CANSparkMax.ControlType.kVelocity, Constants.armVelPIDSlot);
      armTwoPIDController
          .setReference(
              Constants.armFeatherRPM2, CANSparkMax.ControlType.kVelocity, Constants.armVelPIDSlot);
    } else {
      armOnePIDController
          .setReference(
              Constants.armHeightMin, CANSparkMax.ControlType.kPosition, Constants.armPosPIDSlot);
      armTwoPIDController
          .setReference(
              Constants.armHeightMin, CANSparkMax.ControlType.kPosition, Constants.armPosPIDSlot);
    }
  }

  public void deployPoles() {
    poleOnePIDController
        .setReference(Constants.poleHeightDeploy, CANSparkMax.ControlType.kPosition);
    poleTwoPIDController
        .setReference(Constants.poleHeightDeploy, CANSparkMax.ControlType.kPosition);
  }

  public void unlockHooks() {
    hookLocked = false;
    servoOne.set(Constants.climbServo1Unlocked);
    servoTwo.set(Constants.climbServo2Unlocked);
  }

  public void lockHooks() {
    hookLocked = true;
    servoOne.set(Constants.climbServo1Locked);
    servoTwo.set(Constants.climbServo2Locked);
  }

  public void setAutoBoolean(boolean a) {
    BAutomaticControl.setBoolean(a);
  }

  public boolean atFirstSetpoint() {
    // checking if the armHeight is within an acceptable range
    return (armEncoderHeightOne > Constants.armHeightMin - Constants.climbArmDeadband
            && armEncoderHeightOne < Constants.armHeightMin + Constants.climbArmDeadband)
        && (armEncoderHeightTwo > Constants.armHeightMin - Constants.climbArmDeadband
            && armEncoderHeightTwo < Constants.armHeightMin + Constants.climbArmDeadband);
  }

  public void periodic() {
    if (DriverStation.isDisabled() && climbEnabled) {
      toggleEnabled();
    }

    if (DriverStation.isDisabled()) {
      hookLocked = true;
    }
    if (hookLocked) {
      lockHooks();
    }

    BClimbEnabled.setBoolean(climbEnabled);
    DClimbHeight1.setNumber(armOneMotor.getEncoder().getPosition());
    DClimbHeight2.setNumber(armTwoMotor.getEncoder().getPosition());
    DClimbHeight3.setNumber(poleOneMotor.getEncoder().getPosition());
    DClimbHeight4.setNumber(poleTwoMotor.getEncoder().getPosition());

    armEncoderHeightOne = armOneMotor.getEncoder().getPosition();
    armEncoderHeightTwo = armTwoMotor.getEncoder().getPosition();
    poleEncoderHeightOne = poleOneMotor.getEncoder().getPosition();
    poleEncoderHeightTwo = poleTwoMotor.getEncoder().getPosition();

    SmartDashboard.putNumber("arm vel", armOneMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("arm pos", armOneMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("arm output", armOneMotor.getAppliedOutput());
  }
}
