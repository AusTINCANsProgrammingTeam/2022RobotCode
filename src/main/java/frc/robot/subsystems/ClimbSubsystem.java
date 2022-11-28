// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.common.hardware.MotorController;
import frc.robot.common.hardware.MotorController.MotorConfig;

public class ClimbSubsystem extends SubsystemBase {

  // 1 = Right Side, 2 = Left Side
  private CANSparkMax armOneMotor;
  private CANSparkMax armTwoMotor;
  private CANSparkMax poleOneMotor;
  private CANSparkMax poleTwoMotor;

  private RelativeEncoder armOneEncoder;
  private RelativeEncoder armTwoEncoder;
  private RelativeEncoder poleOneEncoder;
  private RelativeEncoder poleTwoEncoder;

  private SparkMaxPIDController armOnePIDController;
  private SparkMaxPIDController armTwoPIDController;
  private SparkMaxPIDController poleOnePIDController;
  private SparkMaxPIDController poleTwoPIDController;

  private Servo servoOne;
  private Servo servoTwo;

  private boolean climbEnabled;

  private double armOnePosition;
  private double armTwoPosition;
  private double poleOnePosition;
  private double poleTwoPosition;

  // Operator Tab
  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry armOneIndicator =
      operatorTab
          .add("Arm 1 Position", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(6, 0)
          .getEntry();
  private NetworkTableEntry armTwoIndicator =
      operatorTab
          .add("Arm 2 Position", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(6, 1)
          .getEntry();
  private NetworkTableEntry poleOneIndicator =
      operatorTab
          .add("Pole 1 Position", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(8, 0)
          .getEntry();
  private NetworkTableEntry poleTwoIndicator =
      operatorTab
          .add("Pole 2 Position", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(8, 1)
          .getEntry();
  private NetworkTableEntry enableIndicator =
      operatorTab
          .add("Climb Enabled", false)
          .withPosition(5, 0)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry();
  private NetworkTableEntry deployIndicator =
      operatorTab
          .add("Climb Deployed", false)
          .withPosition(5, 1)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry();

  public ClimbSubsystem() {
    climbEnabled = false;

    // Arm 1 MotorController
    armOneMotor = MotorController.constructMotor(MotorConfig.climbArmOne);
    armOneMotor.setIdleMode(IdleMode.kBrake);
    armOneEncoder = armOneMotor.getEncoder();
    armOneEncoder.setPosition(0);
    armOnePIDController = MotorController.constructPIDController(armOneMotor, ClimbConstants.armPIDArray);

    // Arm 2 MotorController
    armTwoMotor = MotorController.constructMotor(MotorConfig.climbArmTwo);
    armTwoMotor.setIdleMode(IdleMode.kBrake);
    armTwoEncoder = armTwoMotor.getEncoder();
    armTwoEncoder.setPosition(0);
    armTwoPIDController = MotorController.constructPIDController(armTwoMotor, ClimbConstants.armPIDArray);

    // Pole 1 MotorController
    poleOneMotor = MotorController.constructMotor(MotorConfig.climbPoleOne);
    poleOneMotor.setIdleMode(IdleMode.kBrake);
    poleOneEncoder = poleOneMotor.getEncoder();
    poleOneEncoder.setPosition(0);
    poleOnePIDController = MotorController.constructPIDController(poleOneMotor, ClimbConstants.polePIDArray);

    // Pole 2 MotorController
    poleTwoMotor = MotorController.constructMotor(MotorConfig.climbPoleTwo);
    poleTwoMotor.setIdleMode(IdleMode.kBrake);
    poleTwoEncoder = poleTwoMotor.getEncoder();
    poleTwoEncoder.setPosition(0);
    poleTwoPIDController = MotorController.constructPIDController(poleTwoMotor, ClimbConstants.polePIDArray);

    //PID Settings
    armOnePIDController.setOutputRange(-ClimbConstants.PIDMaxOutput, ClimbConstants.PIDMaxOutput);
    armTwoPIDController.setOutputRange(-ClimbConstants.PIDMaxOutput, ClimbConstants.PIDMaxOutput);
    poleOnePIDController.setOutputRange(-ClimbConstants.PIDMaxOutput, ClimbConstants.PIDMaxOutput);
    poleTwoPIDController.setOutputRange(-ClimbConstants.PIDMaxOutput, ClimbConstants.PIDMaxOutput);

    armOnePIDController.setIMaxAccum(ClimbConstants.armIMaxAccum, 0);
    armTwoPIDController.setIMaxAccum(ClimbConstants.armIMaxAccum, 0);
    poleOnePIDController.setIMaxAccum(ClimbConstants.armIMaxAccum, 0);
    poleTwoPIDController.setIMaxAccum(ClimbConstants.armIMaxAccum, 0);

    // servos
    servoOne = new Servo(ClimbConstants.servoOneID);
    servoTwo = new Servo(ClimbConstants.servoTwoID);

    deployIndicator.setBoolean(false);
    lockHooks();
  }

  public void updateClimbHeights() {
    armOnePosition = armOneEncoder.getPosition();
    armTwoPosition = armTwoEncoder.getPosition();

    poleOnePosition = poleOneEncoder.getPosition();
    poleTwoPosition = poleTwoEncoder.getPosition();
  }

  public void holdPosition() {
    armOnePIDController.setReference(armOnePosition, ControlType.kPosition);
    armTwoPIDController.setReference(armTwoPosition, ControlType.kPosition);
    poleOnePIDController.setReference(poleOnePosition, ControlType.kPosition);
    poleTwoPIDController.setReference(poleTwoPosition, ControlType.kPosition);
  }

  public void toggleEnabled() {
    climbEnabled = !climbEnabled;
    if (climbEnabled) {
      armOneMotor.setSmartCurrentLimit(ClimbConstants.armHighCurrent);
      armTwoMotor.setSmartCurrentLimit(ClimbConstants.armHighCurrent);
      poleOneMotor.setSmartCurrentLimit(ClimbConstants.poleHighCurrent);
      poleTwoMotor.setSmartCurrentLimit(ClimbConstants.poleHighCurrent);
    } else {
      armOneMotor.setSmartCurrentLimit(ClimbConstants.armLowCurrent);
      armTwoMotor.setSmartCurrentLimit(ClimbConstants.armLowCurrent);
      poleOneMotor.setSmartCurrentLimit(ClimbConstants.poleLowCurrent);
      poleTwoMotor.setSmartCurrentLimit(ClimbConstants.poleLowCurrent);
    }
  }

  public void midClimb(double axisValue) {
    if (climbEnabled) {
        if (axisValue > 0) {
          if (armOnePosition + (axisValue * ClimbConstants.armUpSpeed) >= ClimbConstants.armMinPosition) {
            armOnePosition = armOnePosition + (axisValue * ClimbConstants.armUpSpeed);
          }
          if (armTwoPosition + (axisValue * ClimbConstants.armUpSpeed) >= ClimbConstants.armMinPosition) {
            armTwoPosition = armTwoPosition + (axisValue * ClimbConstants.armUpSpeed);
          }
        }
        if (axisValue < 0) {
          if (armOnePosition + (axisValue * ClimbConstants.armDownSpeed) <= ClimbConstants.armMaxPosition) {
            armOnePosition = armOnePosition + (axisValue * ClimbConstants.armDownSpeed);
          }
          if (armTwoPosition + (axisValue * ClimbConstants.armDownSpeed) <= ClimbConstants.armMaxPosition) {
            armTwoPosition = armTwoPosition + (axisValue * ClimbConstants.armDownSpeed);
          }
        }
      }
      armOnePIDController.setReference(armOnePosition, ControlType.kPosition);
      armTwoPIDController.setReference(armTwoPosition, ControlType.kPosition);
    }

  public void highArms(double axisValue) {
      if (axisValue > 0) {
        if (poleOnePosition + (axisValue * ClimbConstants.poleInSpeed) <= ClimbConstants.poleMaxPosition) {
          poleOnePosition = poleOnePosition + (axisValue * ClimbConstants.poleInSpeed);
        }
        if (poleTwoPosition + (axisValue * ClimbConstants.poleInSpeed) <= ClimbConstants.poleMaxPosition) {
          poleTwoPosition = poleTwoPosition + (axisValue * ClimbConstants.poleInSpeed);
        }
      }
      if (axisValue < 0) {
        if (poleOnePosition + (axisValue * ClimbConstants.poleOutSpeed)
            >= ClimbConstants.poleMinPosition) {
          poleOnePosition = poleOnePosition + (axisValue * ClimbConstants.poleOutSpeed);
        }
        if (poleTwoPosition + (axisValue * ClimbConstants.poleOutSpeed)
            >= ClimbConstants.poleMinPosition) {
          poleTwoPosition = poleTwoPosition + (axisValue * ClimbConstants.poleOutSpeed);
        }
      }
    poleOnePIDController.setReference(poleOnePosition, ControlType.kPosition);
    poleTwoPIDController.setReference(poleOnePosition, ControlType.kPosition);
  }
  
  public void retractPoles() {
    poleOnePIDController.setReference(ClimbConstants.poleMaxPosition, ControlType.kPosition);
    poleTwoPIDController.setReference(ClimbConstants.poleMaxPosition, ControlType.kPosition);
  }

  public void deployPoles() {
    poleOnePIDController.setReference(ClimbConstants.poleDeployPosition, ControlType.kPosition);
    poleTwoPIDController.setReference(ClimbConstants.poleDeployPosition, ControlType.kPosition);
  }

  public void unlockHooks() {
    deployIndicator.setBoolean(true);
    servoOne.set(ClimbConstants.servoOneUnlocked);
    servoTwo.set(ClimbConstants.servoTwoUnlocked);
  }

  public void lockHooks() {
    servoOne.set(ClimbConstants.servoOneLocked);
    servoTwo.set(ClimbConstants.servoTwoLocked);
  }

  public void periodic() {
    updateClimbHeights();

    enableIndicator.setBoolean(climbEnabled);
    armOneIndicator.setNumber(armOnePosition);
    armTwoIndicator.setNumber(armTwoPosition);
    poleOneIndicator.setNumber(poleOnePosition);
    poleTwoIndicator.setNumber(poleTwoPosition);

    if (DriverStation.isDisabled() && climbEnabled) {
      toggleEnabled();
    }

    if (DriverStation.isDisabled()) {
      lockHooks();
    }
  }
}
