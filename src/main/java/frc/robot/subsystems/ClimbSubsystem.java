// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;

/** Add your docs here. */
public class ClimbSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Joystick m_climbJoystick;
  private MotorController m_midClimbMotorControllerOne;
  private MotorController m_midClimbMotorControllerTwo;
  private MotorController m_HeighArmsOne;
  private MotorController m_HeighArmsTwo;
  private boolean climbEnabbled;
  private double climbHeightOne;
  private double climbHeightTwo;
  private double joystickAxis;

  private ShuffleboardTab climbTab;

  private NetworkTableEntry sbclimbspeedOne;
  private NetworkTableEntry sbclimbTargettedHightOne;
  private NetworkTableEntry sbclimbHeightOne;
  private NetworkTableEntry sbClimbOneP;
  private NetworkTableEntry sbClimbOneI;
  private NetworkTableEntry sbClimbOneD;

  private NetworkTableEntry sbclimbspeedTwo;
  private NetworkTableEntry sbclimbTargettedHighTwo;
  private NetworkTableEntry sbclimbHeightTwo;
  private NetworkTableEntry sbClimbTwoP;
  private NetworkTableEntry sbClimbTwoI;
  private NetworkTableEntry sbClimbTwoD;

  private NetworkTableEntry sbclimbSpeedInput;
  private NetworkTableEntry sbClimbEnabbled;
  
  private NetworkTableEntry sbhighArmOne;
  private NetworkTableEntry sbhighArmTwo;
  private NetworkTableEntry sbhighArmSpeedOne;
  private NetworkTableEntry sbhighArmSpeedTwo;
  private NetworkTableEntry sbhighArmTargettedHeightOne;
  private NetworkTableEntry sbhighArmTargettedHeightTwo;

  private NetworkTableEntry sbhighArmOneP;
  private NetworkTableEntry sbhighArmOneI;
  private NetworkTableEntry sbhighArmOneD;
  private NetworkTableEntry sbhighArmTwoP;
  private NetworkTableEntry sbhighArmTwoI;
  private NetworkTableEntry sbhighArmTwoD;
  

  public ClimbSubsystem(Joystick joystick) {
    m_climbJoystick = joystick;
    climbEnabbled = false;
    climbHeightOne = 0;
    climbHeightTwo = 0;

    // Mid Climb Arms
    // One is left, two is right
    m_midClimbMotorControllerOne =
        new MotorController(
            "Mid Climb Motor One", Constants.MidClimbMotorOne, Constants.climbLeftPID);
    m_midClimbMotorControllerOne.setSmartCurrentLimit(10);
    m_midClimbMotorControllerTwo =
        new MotorController(
            "Mid Climb Motor Two", Constants.MidClimbMotorTwo, Constants.climbRightPID);
    m_midClimbMotorControllerTwo.setSmartCurrentLimit(10);
    m_midClimbMotorControllerTwo.setInverted(true);
    m_midClimbMotorControllerOne.getEncoder().setPosition(0);
    m_midClimbMotorControllerTwo.getEncoder().setPosition(0);

    m_midClimbMotorControllerOne.setIdleMode(IdleMode.kBrake);
    m_midClimbMotorControllerTwo.setIdleMode(IdleMode.kBrake);

    //Heigh  Arms
    m_HeighArmsOne =
        new MotorController("Traversal Climb Motor One", Constants.TraversalClimbMotorOne);
    m_HeighArmsOne.setSmartCurrentLimit(10);

    m_HeighArmsTwo =
        new MotorController("Traversal Climb Motor Two", Constants.TraversalClimbMotorTwo);
    m_HeighArmsTwo.setSmartCurrentLimit(10);
    m_HeighArmsTwo.setInverted(true);

    // Shuffle Board Widgets
    climbTab = Shuffleboard.getTab("ClimbBase");

    // Climb Arm 1
        climbTab.add("Climb Height 1", 0).withSize(2, 1).withPosition(0, 1).getEntry();
    sbClimbOneP =
        climbTab
            .add("climb One P", Constants.climbRightPID[0])
            .withSize(2, 1)
            .withPosition(0, 2)
            .getEntry();
    sbClimbOneI =
        climbTab
            .add("climb One I", Constants.climbRightPID[1])
            .withSize(2, 1)
            .withPosition(0, 3)
            .getEntry();
    sbClimbOneD =
        climbTab
            .add("climb One D", Constants.climbRightPID[2])
            .withSize(2, 1)
            .withPosition(0, 4)
            .getEntry();
    sbclimbTargettedHightOne =
        climbTab.add("Climb targetted height 1", 0).withSize(2, 2).withPosition(2, 2).getEntry();
    sbclimbspeedOne =
        climbTab.add("Climb Current Speed 1", 0).withSize(2, 1).withPosition(2, 4).getEntry();

    // Climb Arm 2
    sbclimbHeightTwo =
        climbTab.add("Climb Height 2", 0).withSize(2, 1).withPosition(8, 1).getEntry();
    sbClimbTwoP =
        climbTab
            .add("climb Two P", Constants.climbLeftPID[0])
            .withSize(2, 1)
            .withPosition(8, 2)
            .getEntry();
    sbClimbTwoI =
        climbTab
            .add("climb Two I", Constants.climbLeftPID[1])
            .withSize(2, 1)
            .withPosition(8, 3)
            .getEntry();
    sbClimbTwoD =
        climbTab
            .add("climb Two D", Constants.climbLeftPID[2])
            .withSize(2, 1)
            .withPosition(8, 4)
            .getEntry();
    sbclimbTargettedHighTwo =
        climbTab.add("Climb targetted height 2", 0).withSize(2, 2).withPosition(6, 2).getEntry();
    sbclimbspeedTwo =
        climbTab.add("climb Current Speed 2", 0).withSize(2, 1).withPosition(6, 4).getEntry();

    // Both Arms
    sbClimbEnabbled =
        climbTab.add("Climb Eanbled", false).withSize(3, 2).withPosition(2, 0).getEntry();
    sbclimbSpeedInput =
        climbTab.add("Climb Speed input", 0.1).withSize(2, 3).withPosition(4, 2).getEntry();
    sbHandPosition = climbTab.add("Traversal Hand position",0).withSize(5, 4).getEntry();
  }

  public void resetTargetedHeight() {
    climbHeightOne = m_midClimbMotorControllerOne.getEncoder().getPosition();
    climbHeightTwo = m_midClimbMotorControllerTwo.getEncoder().getPosition();
  }

  public void climbKeepDownFunction() {
    m_midClimbMotorControllerOne
        .getPIDCtrl()
        .setReference(climbHeightOne, CANSparkMax.ControlType.kPosition);
    sbclimbHeightOne.setNumber(climbHeightOne);

    m_midClimbMotorControllerTwo
        .getPIDCtrl()
        .setReference(climbHeightTwo, CANSparkMax.ControlType.kPosition);
    sbclimbHeightTwo.setNumber(climbHeightTwo);
  }

  public void climbEnable() {
    climbEnabbled = !climbEnabbled;
    sbClimbEnabbled.setBoolean(climbEnabbled);
    if (climbEnabbled) {
      m_midClimbMotorControllerOne.setSmartCurrentLimit(60);
      m_midClimbMotorControllerTwo.setSmartCurrentLimit(60);
      m_HeighArmsOne.setSmartCurrentLimit(60);
      m_HeighArmsTwo.setSmartCurrentLimit(60);
    } else {
      m_midClimbMotorControllerOne.setSmartCurrentLimit(10);
      m_midClimbMotorControllerTwo.setSmartCurrentLimit(10);
      m_HeighArmsOne.setSmartCurrentLimit(10);
      m_HeighArmsTwo.setSmartCurrentLimit(10);
    }
  }

  public boolean getclimbingenable() {
    return sbClimbEnabbled.getBoolean(false);
  }

  public void runManual() {
    if (climbEnabbled) {

      joystickAxis = -m_climbJoystick.getRawAxis(Constants.leftJoystickY);
      if (joystickAxis > 0.1 || joystickAxis < -0.1) {
        if (joystickAxis > 0) {
          m_midClimbMotorControllerOne.set(sbclimbSpeedInput.getDouble(0));
          m_midClimbMotorControllerTwo.set(sbclimbSpeedInput.getDouble(0));
        }
        if (joystickAxis < 0) {
          m_midClimbMotorControllerOne.set(-sbclimbSpeedInput.getDouble(0));
          m_midClimbMotorControllerTwo.set(-sbclimbSpeedInput.getDouble(0));
        }
      } else {
        m_midClimbMotorControllerOne.set(0);
        m_midClimbMotorControllerTwo.set(0);
      }
    }
  }

  public void enableClimb() {
    if (climbEnabbled) {

      joystickAxis = -m_climbJoystick.getRawAxis(Constants.leftJoystickY);
      if (joystickAxis > 0.1 || joystickAxis < -0.1) {
        if (joystickAxis > 0) {
          if (climbHeightOne + (joystickAxis / 10 * 8) <= Constants.climbHeightMax) {
            climbHeightOne = climbHeightOne + (joystickAxis / 10 * 15);
          }
          if (climbHeightTwo + (joystickAxis / 10 * 8) <= Constants.climbHeightMax) {
            climbHeightTwo = climbHeightTwo + (joystickAxis / 10 * 15);
          }
        }
        if (joystickAxis < 0) {
          if (climbHeightOne + (joystickAxis / 10 * 6) >= 0) {
            climbHeightOne = climbHeightOne + (joystickAxis / 10 * 6);
          }
          if (climbHeightTwo + (joystickAxis / 10 * 6) >= 0) {
            climbHeightTwo = climbHeightTwo + (joystickAxis / 10 * 6);
          }
        }
      }
      m_midClimbMotorControllerOne
          .getPIDCtrl()
          .setReference(climbHeightOne, CANSparkMax.ControlType.kPosition);
      sbclimbHeightOne.setNumber(climbHeightOne);

      m_midClimbMotorControllerTwo
          .getPIDCtrl()
          .setReference(climbHeightTwo, CANSparkMax.ControlType.kPosition);
      sbclimbHeightTwo.setNumber(climbHeightTwo);
    } else {
      // m_climbMotorControllerOne.getPID().setReference(0,
      // CANSparkMax.ControlType.kVoltage);
    }
  }

  public void periodic() {
    if (DriverStation.isDisabled() && climbEnabbled) {
      climbEnable();
    }

    SmartDashboard.putNumber(
        "Climb motor 1 Applied Output", m_midClimbMotorControllerOne.getAppliedOutput());
    SmartDashboard.putNumber(
        "Climb motor 2 Applied Output", m_midClimbMotorControllerTwo.getAppliedOutput());
    SmartDashboard.putNumber(
        "Climb Hight One", m_midClimbMotorControllerOne.getEncoder().getPosition());
    if (sbclimbHeightOne.getDouble(0) != climbHeightOne) {
      climbHeightOne = sbclimbHeightOne.getDouble(0);
    } else {
      sbclimbTargettedHightOne.setDouble(climbHeightOne);
    }
    sbclimbspeedOne.setDouble(m_midClimbMotorControllerOne.getEncoder().getVelocity());

    // m_climbMotorControllerOne.updateSmartDashboard();
    SmartDashboard.putNumber(
        "Climb IAccum One", m_midClimbMotorControllerOne.getPIDCtrl().getIAccum());

    SmartDashboard.putNumber(
        "Climb Hight Two", m_midClimbMotorControllerTwo.getEncoder().getPosition());
    if (sbclimbHeightTwo.getDouble(0) != climbHeightTwo) {
      climbHeightTwo = sbclimbHeightTwo.getDouble(0);
    } else {
      sbclimbTargettedHighTwo.setDouble(climbHeightTwo);
    }
    sbclimbspeedTwo.setDouble(m_midClimbMotorControllerTwo.getEncoder().getVelocity());

    // m_climbMotorControllerTwo.updateSmartDashboard();
    SmartDashboard.putNumber(
        "Climb IAccum Two", m_midClimbMotorControllerTwo.getPIDCtrl().getIAccum());
    if ((m_midClimbMotorControllerOne.getPIDCtrl().getP() != sbClimbOneP.getDouble(0))
        || (m_midClimbMotorControllerOne.getPIDCtrl().getI() != sbClimbOneI.getDouble(0))
        || (m_midClimbMotorControllerOne.getPIDCtrl().getD() != sbClimbOneD.getDouble(0))) {
      m_midClimbMotorControllerOne.getPIDCtrl().setP(sbClimbOneP.getDouble(0));
      m_midClimbMotorControllerOne.getPIDCtrl().setI(sbClimbOneI.getDouble(0));
      m_midClimbMotorControllerOne.getPIDCtrl().setD(sbClimbOneD.getDouble(0));
    }

    if ((m_midClimbMotorControllerTwo.getPIDCtrl().getP() != sbClimbTwoP.getDouble(0))
        || (m_midClimbMotorControllerTwo.getPIDCtrl().getI() != sbClimbTwoI.getDouble(0))
        || (m_midClimbMotorControllerTwo.getPIDCtrl().getD() != sbClimbTwoD.getDouble(0))) {
      m_midClimbMotorControllerOne.getPIDCtrl().setP(sbClimbOneP.getDouble(0));
      m_midClimbMotorControllerOne.getPIDCtrl().setI(sbClimbOneI.getDouble(0));
      m_midClimbMotorControllerOne.getPIDCtrl().setD(sbClimbOneD.getDouble(0));
    }
  }
}
