// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;

public class ClimbSubsystem extends SubsystemBase {

  // 1 = Right Side, 2 = Left Side
  private Joystick climbJoystick;
  private MotorController armOne;
  private MotorController armTwo;
  private MotorController poleOne;
  private MotorController poleTwo;
  private Servo servoOne;
  private Servo servoTwo;

  private boolean climbEnable, hookLocked;

  private double armHeightOne;
  private double armHeightTwo;
  private double poleHeightOne;
  private double poleHeightTwo;
  private double armEncoderHeightOne;
  private double armEncoderHeightTwo;
  private double poleEncoderHeightOne;
  private double poleEncoderHeightTwo;

  private double armJoystickAxis;
  private double poleJoystickAxis;

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

  public ClimbSubsystem(Joystick joystick) {
    climbJoystick = joystick;
    climbEnable = false;

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
    armOne = new MotorController("Arm1 Motor", Constants.armMotorOne, Constants.armPosPID);
    armOne.setSmartCurrentLimit(10);
    armOne.getEncoder().setPosition(0);
    armOne.setIdleMode(IdleMode.kBrake);

    // Arm 2 MotorController
    armTwo = new MotorController("Arm2 Motor", Constants.armMotorTwo, Constants.armPosPID);
    armTwo.setSmartCurrentLimit(10);
    armTwo.getEncoder().setPosition(0);
    armTwo.setIdleMode(IdleMode.kBrake);
    armTwo.setInverted(true);
    // Pole 1 MotorController
    poleOne = new MotorController("Pole1 Motor", Constants.poleMotorOne, Constants.polePosPID);
    poleOne.setSmartCurrentLimit(10);
    poleOne.getEncoder().setPosition(0);
    poleOne.setIdleMode(IdleMode.kBrake);

    // Pole 2 MotorController
    poleTwo = new MotorController("Pole2 Motor", Constants.poleMotorTwo, Constants.polePosPID);
    poleTwo.setSmartCurrentLimit(10);
    poleTwo.getEncoder().setPosition(0);
    poleTwo.setIdleMode(IdleMode.kBrake);
    poleTwo.setInverted(true);

    // set the max output on each pid controller
    // range is plus minus the max output
    armOne.getPIDCtrl().setOutputRange(-Constants.climbMaxOutput, Constants.climbMaxOutput);
    armTwo.getPIDCtrl().setOutputRange(-Constants.climbMaxOutput, Constants.climbMaxOutput);
    poleOne.getPIDCtrl().setOutputRange(-Constants.climbMaxOutput, Constants.climbMaxOutput);
    poleTwo.getPIDCtrl().setOutputRange(-Constants.climbMaxOutput, Constants.climbMaxOutput);

    // set pid values
    armOne.getPIDCtrl().setP(Constants.armVelocityPID[0], Constants.armVelPIDSlot);
    armOne.getPIDCtrl().setI(Constants.armVelocityPID[1], Constants.armVelPIDSlot);
    armOne.getPIDCtrl().setD(Constants.armVelocityPID[2], Constants.armVelPIDSlot);

    armTwo.getPIDCtrl().setP(Constants.armVelocityPID[0], Constants.armVelPIDSlot);
    armTwo.getPIDCtrl().setI(Constants.armVelocityPID[1], Constants.armVelPIDSlot);
    armTwo.getPIDCtrl().setD(Constants.armVelocityPID[2], Constants.armVelPIDSlot);

    armOne.getPIDCtrl().setP(Constants.armPosPID[0], Constants.armPosPIDSlot);
    armOne.getPIDCtrl().setI(Constants.armPosPID[1], Constants.armPosPIDSlot);
    armOne.getPIDCtrl().setD(Constants.armPosPID[2], Constants.armPosPIDSlot);

    armTwo.getPIDCtrl().setP(Constants.armPosPID[0], Constants.armPosPIDSlot);
    armTwo.getPIDCtrl().setI(Constants.armPosPID[1], Constants.armPosPIDSlot);
    armTwo.getPIDCtrl().setD(Constants.armPosPID[2], Constants.armPosPIDSlot);

    resetClimbHeights();

    // servos
    servoOne = new Servo(Constants.climbServoIDOne);
    servoTwo = new Servo(Constants.climbServoIDTwo);

    lockHooks();
  }

  public void resetClimbHeights() {
    armHeightOne = armOne.getEncoder().getPosition();
    armHeightTwo = armTwo.getEncoder().getPosition();

    poleHeightOne = poleOne.getEncoder().getPosition();
    poleHeightTwo = poleTwo.getEncoder().getPosition();
  }

  public void climbKeepDownFunction() {
    armOne.getPIDCtrl().setReference(armHeightOne, CANSparkMax.ControlType.kPosition);
    armOne.getPIDCtrl().setIMaxAccum(Constants.armSetIMaxAccum, 0);

    armTwo.getPIDCtrl().setReference(armHeightTwo, CANSparkMax.ControlType.kPosition);
    armTwo.getPIDCtrl().setIMaxAccum(Constants.armSetIMaxAccum, 0);

    poleOne.getPIDCtrl().setReference(poleHeightOne, CANSparkMax.ControlType.kPosition);
    poleOne.getPIDCtrl().setIMaxAccum(Constants.poleSetIMaxAccum, 0);

    poleTwo.getPIDCtrl().setReference(poleHeightTwo, CANSparkMax.ControlType.kPosition);
    poleTwo.getPIDCtrl().setIMaxAccum(Constants.poleSetIMaxAccum, 0);
  }

  public void climbEnable() {
    climbEnable = !climbEnable;
    if (climbEnable) {
      armOne.setSmartCurrentLimit(Constants.climbArmHighCurrent);
      armTwo.setSmartCurrentLimit(Constants.climbArmHighCurrent);
      poleOne.setSmartCurrentLimit(Constants.climbPoleHighCurrent);
      poleTwo.setSmartCurrentLimit(Constants.climbPoleHighCurrent);
    } else {
      armOne.setSmartCurrentLimit(Constants.climbArmLowCurrent);
      armTwo.setSmartCurrentLimit(Constants.climbArmLowCurrent);
      poleOne.setSmartCurrentLimit(Constants.climbPoleLowCurrent);
      poleTwo.setSmartCurrentLimit(Constants.climbPoleLowCurrent);
    }
  }

  public boolean getclimbingenable() {
    return climbEnable;
  }

  public void midClimb() {
    if (climbEnable) {
      armJoystickAxis = -climbJoystick.getRawAxis(Constants.leftJoystickY);
      if (armJoystickAxis > Constants.controllerDeadZone
          || armJoystickAxis < -Constants.controllerDeadZone) {
        if (armJoystickAxis > 0) {
          /*if (armHeightOne + (armJoystickAxis * (Constants.ArmUpSpeed/2)) >= Constants.ArmHeightFeather) {
            armHeightOne = armHeightOne + (armJoystickAxis * Constants.ArmUpSpeed/2);
          } else{*/
          if (armHeightOne + (armJoystickAxis * Constants.armUpSpeed) >= Constants.armHeightMin) {
            armHeightOne = armHeightOne + (armJoystickAxis * Constants.armUpSpeed);
            // }
          }
          /*if (armHeightTwo + (armJoystickAxis * (Constants.ArmUpSpeed/2)) >= Constants.ArmHeightFeather) {
            armHeightTwo = armHeightTwo + (armJoystickAxis * Constants.ArmUpSpeed/2);
          } else{*/
          if (armHeightTwo + (armJoystickAxis * Constants.armUpSpeed) >= Constants.armHeightMin) {
            armHeightTwo = armHeightTwo + (armJoystickAxis * Constants.armUpSpeed);
            // }
          }
        }
        if (armJoystickAxis < 0) {
          if (armHeightOne + (armJoystickAxis * Constants.armDownSpeed) <= Constants.armHeightMax) {
            armHeightOne = armHeightOne + (armJoystickAxis * Constants.armDownSpeed);
          }
          if (armHeightTwo + (armJoystickAxis * Constants.armDownSpeed) <= Constants.armHeightMax) {
            armHeightTwo = armHeightTwo + (armJoystickAxis * Constants.armDownSpeed);
          }
        }
      }
      armOne.getPIDCtrl().setReference(armHeightOne, CANSparkMax.ControlType.kPosition);

      armTwo.getPIDCtrl().setReference(armHeightTwo, CANSparkMax.ControlType.kPosition);
    }
  }

  public void highArms() {
      poleJoystickAxis = -climbJoystick.getRawAxis(Constants.rightJoystickY);
      if (poleJoystickAxis > Constants.controllerDeadZone
          || poleJoystickAxis < -Constants.controllerDeadZone) {
        if (poleJoystickAxis > 0) {
          if (poleHeightOne + (poleJoystickAxis * Constants.poleInSpeed)
              <= Constants.poleHeightMax) {
            poleHeightOne = poleHeightOne + (poleJoystickAxis * Constants.poleInSpeed);
          }
          if (poleHeightTwo + (poleJoystickAxis * Constants.poleInSpeed)
              <= Constants.poleHeightMax) {
            poleHeightTwo = poleHeightTwo + (poleJoystickAxis * Constants.poleInSpeed);
          }
        }
        if (poleJoystickAxis < 0) {
          if (poleHeightOne + (poleJoystickAxis * Constants.poleOutSpeed)
              >= Constants.poleHeightMin) {
            poleHeightOne = poleHeightOne + (poleJoystickAxis * Constants.poleOutSpeed);
          }
          if (poleHeightTwo + (poleJoystickAxis * Constants.poleOutSpeed)
              >= Constants.poleHeightMin) {
            poleHeightTwo = poleHeightTwo + (poleJoystickAxis * Constants.poleOutSpeed);
          }
        }
      }
      poleOne.getPIDCtrl().setReference(poleHeightOne, CANSparkMax.ControlType.kPosition);

      poleTwo.getPIDCtrl().setReference(poleHeightOne, CANSparkMax.ControlType.kPosition);
  }

  // called at the start of auton
  public void retractPoles() {
    poleOne.getPIDCtrl().setReference(Constants.poleHeightMax, CANSparkMax.ControlType.kPosition);
    poleTwo.getPIDCtrl().setReference(Constants.poleHeightMax, CANSparkMax.ControlType.kPosition);
  }

  // functions for ClimbSequence1
  public void deployArms() {
    if (armEncoderHeightOne > Constants.armHeightFeather1
        || armEncoderHeightTwo > Constants.armHeightFeather1) {
      armOne
          .getPIDCtrl()
          .setReference(
              Constants.armFeatherRPM1, CANSparkMax.ControlType.kVelocity, Constants.armVelPIDSlot);
      armTwo
          .getPIDCtrl()
          .setReference(
              Constants.armFeatherRPM1, CANSparkMax.ControlType.kVelocity, Constants.armVelPIDSlot);
    } else if (armEncoderHeightOne > Constants.armHeightFeather2
        || armEncoderHeightTwo > Constants.armHeightFeather2) {
      armOne
          .getPIDCtrl()
          .setReference(
              Constants.armFeatherRPM2, CANSparkMax.ControlType.kVelocity, Constants.armVelPIDSlot);
      armTwo
          .getPIDCtrl()
          .setReference(
              Constants.armFeatherRPM2, CANSparkMax.ControlType.kVelocity, Constants.armVelPIDSlot);
    } else {
      armOne
          .getPIDCtrl()
          .setReference(
              Constants.armHeightMin, CANSparkMax.ControlType.kPosition, Constants.armPosPIDSlot);
      armTwo
          .getPIDCtrl()
          .setReference(
              Constants.armHeightMin, CANSparkMax.ControlType.kPosition, Constants.armPosPIDSlot);
    }
  }

  public void deployPoles() {
    poleOne
        .getPIDCtrl()
        .setReference(Constants.poleHeightDeploy, CANSparkMax.ControlType.kPosition);
    poleTwo
        .getPIDCtrl()
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
    if (DriverStation.isDisabled() && climbEnable) {
      climbEnable();
    }

    if (DriverStation.isDisabled()) {
      hookLocked = true;
    }
    if (hookLocked) {
      lockHooks();
    }

    BClimbEnabled.setBoolean(climbEnable);
    DClimbHeight1.setNumber(armOne.getEncoder().getPosition());
    DClimbHeight2.setNumber(armTwo.getEncoder().getPosition());
    DClimbHeight3.setNumber(poleOne.getEncoder().getPosition());
    DClimbHeight4.setNumber(poleTwo.getEncoder().getPosition());

    armEncoderHeightOne = armOne.getEncoder().getPosition();
    armEncoderHeightTwo = armTwo.getEncoder().getPosition();
    poleEncoderHeightOne = poleOne.getEncoder().getPosition();
    poleEncoderHeightTwo = poleTwo.getEncoder().getPosition();

    SmartDashboard.putNumber("arm vel", armOne.getEncoder().getVelocity());
    SmartDashboard.putNumber("arm pos", armOne.getEncoder().getPosition());
    SmartDashboard.putNumber("arm output", armOne.getAppliedOutput());

    armOne.updateSmartDashboard();
    armTwo.updateSmartDashboard();
    poleOne.updateSmartDashboard();
    poleTwo.updateSmartDashboard();
  }
}
