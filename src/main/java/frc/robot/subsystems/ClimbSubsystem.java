// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
  private MotorController m_climbMotorControllerOne;
  private MotorController m_climbMotorControllerTwo;
  private DigitalInput m_limitSwitch;
  private boolean climbEnabbled;
  private double climbHeightOne;
  private double climbHeightTwo;
  private double joystickAxis;
  private ShuffleboardTab climbTab;
  private NetworkTableEntry sbclimbpositionOne;
  private NetworkTableEntry sbclimbspeedOne;
  private NetworkTableEntry sbclimbheightOne;
  private NetworkTableEntry sbclimbHeightOne;
  private NetworkTableEntry sbclimbpositionTwo;
  private NetworkTableEntry sbclimbspeedTwo;
  private NetworkTableEntry sbclimbheightTwo;
  private NetworkTableEntry sbclimbHeightTwo;
  private NetworkTableEntry sbClimbOneP;
  private NetworkTableEntry sbClimbOneI;
  private NetworkTableEntry sbClimbOneD;
  private NetworkTableEntry sbClimbTwoP;
  private NetworkTableEntry sbClimbTwoI;
  private NetworkTableEntry sbClimbTwoD;
  private NetworkTableEntry sbclimbSpeedInput;
  private NetworkTableEntry sbclimbMode;
  private NetworkTableEntry sbClimbEnabbled;

  public ClimbSubsystem(Joystick joystick) {
    m_climbJoystick = joystick;
    climbEnabbled = false;
    climbHeightOne = 0;
    climbHeightTwo = 0;

    // One is left, two is right

    m_climbMotorControllerOne =
        new MotorController("Climb Motor One", Constants.ClimbMotorOne, Constants.climbLeftPID);
    m_climbMotorControllerOne.setSmartCurrentLimit(60);
    m_climbMotorControllerTwo =
        new MotorController("Climb Motor Two", Constants.ClimbMotorTwo, Constants.climbRightPID);
    m_climbMotorControllerTwo.setSmartCurrentLimit(60);
    m_climbMotorControllerTwo.setInverted(true);
    m_climbMotorControllerOne.getEncoder().setPosition(0);
    m_climbMotorControllerTwo.getEncoder().setPosition(0);

    m_climbMotorControllerOne.setIdleMode(IdleMode.kBrake);
    m_climbMotorControllerTwo.setIdleMode(IdleMode.kBrake);

    // m_limitSwitch = new DigitalInput(Constants.LimitSwitchChannel);

    // Shuffle Board Widgets
    climbTab = Shuffleboard.getTab("ClimbBase");

    // Climb Arm 1
    sbclimbpositionOne =
        climbTab.add("Climb position 1", 0).withSize(2, 1).withPosition(0, 0).getEntry();
    sbclimbHeightOne =
        climbTab.add("Climb Hight 1", 0).withSize(2, 1).withPosition(0, 1).getEntry();
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
    sbclimbheightOne =
        climbTab.add("Climb targetted height 1", 0).withSize(2, 2).withPosition(2, 2).getEntry();
    sbclimbspeedOne =
        climbTab.add("Climb Current Speed 1", 0).withSize(2, 1).withPosition(2, 4).getEntry();

    // Climb Arm 2
    sbclimbpositionTwo =
        climbTab.add("climb position 2", 0).withSize(2, 1).withPosition(8, 0).getEntry();
    sbclimbHeightTwo =
        climbTab.add("Climb Hight 2", 0).withSize(2, 1).withPosition(8, 1).getEntry();
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
    sbclimbheightTwo =
        climbTab.add("Climb targetted height 2", 0).withSize(2, 2).withPosition(6, 2).getEntry();
    sbclimbspeedTwo =
        climbTab.add("climb Current Speed 2", 0).withSize(2, 1).withPosition(6, 4).getEntry();

    // Both Arms
    sbClimbEnabbled =
        climbTab.add("Climb Eanbled", false).withSize(3, 2).withPosition(2, 0).getEntry();
    sbclimbMode =
        climbTab
            .add("Manual Mode Enable", false)
            .withSize(3, 2)
            .withPosition(5, 0)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
    sbclimbSpeedInput =
        climbTab.add("Climb Speed input", 0.1).withSize(2, 3).withPosition(4, 2).getEntry();
  }

  public void resetTargetedHeight() {
    climbHeightOne = m_climbMotorControllerOne.getEncoder().getPosition();
    climbHeightTwo = m_climbMotorControllerTwo.getEncoder().getPosition();
  }

  public void climbEnable() {
    climbEnabbled = !climbEnabbled;
    sbClimbEnabbled.setBoolean(climbEnabbled);
  }

  public boolean getclimbingmode() {
    return sbclimbMode.getBoolean(false);
  }

  public boolean getclimbingenable() {
    return sbClimbEnabbled.getBoolean(false);
  }

  public void runManual() {
    if (climbEnabbled
    /** && !m_limitSwitch.get() */
    ) {

      joystickAxis = -m_climbJoystick.getRawAxis(Constants.leftJoystickY);
      if (joystickAxis > 0.1 || joystickAxis < -0.1) {
        if (joystickAxis > 0) {
          m_climbMotorControllerOne.set(sbclimbSpeedInput.getDouble(0));
          m_climbMotorControllerTwo.set(sbclimbSpeedInput.getDouble(0));
        }
        if (joystickAxis < 0) {
          m_climbMotorControllerOne.set(-sbclimbSpeedInput.getDouble(0));
          m_climbMotorControllerTwo.set(-sbclimbSpeedInput.getDouble(0));
        }
      } else {
        m_climbMotorControllerOne.set(0);
        m_climbMotorControllerTwo.set(0);
      }
    }
  }

  public void enableClimb() {
    if (climbEnabbled
    /** && !m_limitSwitch.get() */
    ) {

      joystickAxis = -m_climbJoystick.getRawAxis(Constants.leftJoystickY);
      if (joystickAxis > 0.1 || joystickAxis < -0.1) {
        if (joystickAxis > 0) {
          if (climbHeightOne + (joystickAxis / 10 * 8) <= Constants.climbHeightMax) {
            climbHeightOne = climbHeightOne + (joystickAxis / 10 * 8);
          }
          if (climbHeightTwo + (joystickAxis / 10 * 8) <= Constants.climbHeightMax) {
            climbHeightTwo = climbHeightTwo + (joystickAxis / 10 * 8);
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
      m_climbMotorControllerOne
          .getPIDCtrl()
          .setReference(climbHeightOne, CANSparkMax.ControlType.kPosition);
      sbclimbHeightOne.setNumber(climbHeightOne);

      m_climbMotorControllerTwo
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
        "Climb motor 1 Applied Output", m_climbMotorControllerOne.getAppliedOutput());
    SmartDashboard.putNumber(
        "Climb motor 2 Applied Output", m_climbMotorControllerTwo.getAppliedOutput());
    SmartDashboard.putNumber(
        "Climb Hight One", m_climbMotorControllerOne.getEncoder().getPosition());
    if (sbclimbHeightOne.getDouble(0) != climbHeightOne) {
      climbHeightOne = sbclimbHeightOne.getDouble(0);
    } else {
      sbclimbheightOne.setDouble(climbHeightOne);
    }
    sbclimbspeedOne.setDouble(m_climbMotorControllerOne.getEncoder().getVelocity());
    sbclimbpositionOne.setDouble(m_climbMotorControllerOne.getEncoder().getPosition());

    // m_climbMotorControllerOne.updateSmartDashboard();
    SmartDashboard.putNumber(
        "Climb IAccum One", m_climbMotorControllerOne.getPIDCtrl().getIAccum());

    SmartDashboard.putNumber(
        "Climb Hight Two", m_climbMotorControllerTwo.getEncoder().getPosition());
    if (sbclimbHeightTwo.getDouble(0) != climbHeightTwo) {
      climbHeightTwo = sbclimbHeightTwo.getDouble(0);
    } else {
      sbclimbheightTwo.setDouble(climbHeightTwo);
    }
    sbclimbspeedTwo.setDouble(m_climbMotorControllerTwo.getEncoder().getVelocity());
    sbclimbpositionTwo.setDouble(m_climbMotorControllerTwo.getEncoder().getPosition());

    // m_climbMotorControllerTwo.updateSmartDashboard();
    SmartDashboard.putNumber(
        "Climb IAccum Two", m_climbMotorControllerTwo.getPIDCtrl().getIAccum());
    if ((m_climbMotorControllerOne.getPIDCtrl().getP() != sbClimbOneP.getDouble(0))
        || (m_climbMotorControllerOne.getPIDCtrl().getI() != sbClimbOneI.getDouble(0))
        || (m_climbMotorControllerOne.getPIDCtrl().getD() != sbClimbOneD.getDouble(0))) {
      m_climbMotorControllerOne.getPIDCtrl().setP(sbClimbOneP.getDouble(0));
      m_climbMotorControllerOne.getPIDCtrl().setI(sbClimbOneI.getDouble(0));
      m_climbMotorControllerOne.getPIDCtrl().setD(sbClimbOneD.getDouble(0));
    }

    if ((m_climbMotorControllerTwo.getPIDCtrl().getP() != sbClimbTwoP.getDouble(0))
        || (m_climbMotorControllerTwo.getPIDCtrl().getI() != sbClimbTwoI.getDouble(0))
        || (m_climbMotorControllerTwo.getPIDCtrl().getD() != sbClimbTwoD.getDouble(0))) {
      m_climbMotorControllerOne.getPIDCtrl().setP(sbClimbOneP.getDouble(0));
      m_climbMotorControllerOne.getPIDCtrl().setI(sbClimbOneI.getDouble(0));
      m_climbMotorControllerOne.getPIDCtrl().setD(sbClimbOneD.getDouble(0));
    }
  }

  public boolean getLimitSwitchVal() {
    return m_limitSwitch.get();
  }

  // TODO: might add other getter methods depending on how many limit switches
}
