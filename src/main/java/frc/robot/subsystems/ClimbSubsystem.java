// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
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
  private MotorController m_McOne;
  private MotorController m_McTwo;
  private MotorController m_HaOne;
  private MotorController m_HaTwo;
  private boolean climbEnabble;
  private double McHeightOne;
  private double McHeightTwo;
  private double HaHeightOne;
  private double HaHeightTwo;

  private double McjoystickAxis;
  private double HajoystickAxis;

  private ShuffleboardTab climbTab;

  // Mc = Mid Climb
  private NetworkTableEntry sbMcSpeedOne;
  private NetworkTableEntry sbMcTargettedOne;
  private NetworkTableEntry sbMcHeightOne;
  private NetworkTableEntry sbMcOneP;
  private NetworkTableEntry sbMcOneI;
  private NetworkTableEntry sbMcOneD;

  private NetworkTableEntry sbMcSpeedTwo;
  private NetworkTableEntry sbMcTargettedTwo;
  private NetworkTableEntry sbMcHeightTwo;
  private NetworkTableEntry sbMcTwoP;
  private NetworkTableEntry sbMcTwoI;
  private NetworkTableEntry sbMcTwoD;

  // HA = High Arms
  private NetworkTableEntry sbHaHeightOne;
  private NetworkTableEntry sbHaSpeedOne;
  private NetworkTableEntry sbHaTargettedOne;
  private NetworkTableEntry sbHaOneP;
  private NetworkTableEntry sbHaOneI;
  private NetworkTableEntry sbHaOneD;

  private NetworkTableEntry sbHaHeightTwo;
  private NetworkTableEntry sbHaSpeedTwo;
  private NetworkTableEntry sbHaTargettedTwo;
  private NetworkTableEntry sbHaTwoP;
  private NetworkTableEntry sbHaTwoI;
  private NetworkTableEntry sbHaTwoD;

  // Other
  private boolean Organization;
  private NetworkTableEntry sbClimbSpeedInput;
  private NetworkTableEntry sbClimbEnabble;

  // Operator Tab
  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry DClimbHeight1 =
      operatorTab
          .add("Climb Height 1", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(6, 0)
          .getEntry();
  private NetworkTableEntry DClimbHeight2 =
      operatorTab
          .add("Climb Height 2", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(6, 1)
          .getEntry();
  private NetworkTableEntry BClimbEnabled =
      operatorTab
          .add("Climb Enabled", false)
          .withPosition(5, 0)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry();

  public ClimbSubsystem(Joystick joystick) {
    if (Constants.DebugMode) {
      instantiateDebugTab();
    }
    m_climbJoystick = joystick;
    Organization = true;
    climbEnabble = false;

    // Mid Climb Arms
    // One is left, two is right

    // Heigh  Arms
    m_HaOne = new MotorController("Ha1 MotorControllor", Constants.HaMotorOne);
    m_HaOne.setSmartCurrentLimit(10);
    m_HaOne.getEncoder().setPosition(0);

    m_HaTwo = new MotorController("Ha2 MotorControllor", Constants.HaMotorTwo);
    m_HaTwo.setSmartCurrentLimit(10);
    m_HaTwo.getEncoder().setPosition(0);
    m_HaTwo.setInverted(true);

    m_McOne = new MotorController("Climb Motor One", Constants.McMotorOne, Constants.McLeftPID);
    m_McOne.setSmartCurrentLimit(10);
    m_McOne.getEncoder().setPosition(0);
    m_McOne.setIdleMode(IdleMode.kBrake);

    m_McTwo = new MotorController("Climb Motor Two", Constants.McMotorTwo, Constants.McRightPID);
    m_McTwo.setSmartCurrentLimit(10);
    m_McTwo.setInverted(true);
    m_McTwo.getEncoder().setPosition(0);
    m_McTwo.setIdleMode(IdleMode.kBrake);

    resetTargetedHeight();
    // m_climbMotorControllerTwo.getPID().setOutputRange(-.4, .4);
    // m_climbMotorControllerOne.getPID().setOutputRange(-.4, .4);
  }

  public void resetTargetedHeight() {
    McHeightOne = m_McOne.getEncoder().getPosition();
    McHeightTwo = m_McTwo.getEncoder().getPosition();

    HaHeightOne = m_HaOne.getEncoder().getPosition();
    HaHeightTwo = m_HaTwo.getEncoder().getPosition();
  }

  public void climbKeepDownFunction() {
    m_McOne.getPIDCtrl().setReference(McHeightOne, CANSparkMax.ControlType.kPosition);
    sbMcHeightOne.setNumber(McHeightOne);

    m_McTwo.getPIDCtrl().setReference(McHeightTwo, CANSparkMax.ControlType.kPosition);
    sbMcHeightTwo.setNumber(McHeightTwo);

    m_HaOne.getPIDCtrl().setReference(HaHeightOne, CANSparkMax.ControlType.kPosition);
    // Insert High Arm Shuffleboard after done

    m_HaTwo.getPIDCtrl().setReference(HaHeightTwo, CANSparkMax.ControlType.kPosition);
  }

  public void climbEnable() {
    climbEnabble = !climbEnabble;
    if (climbEnabble) {
      m_McOne.setSmartCurrentLimit(60);
      m_McTwo.setSmartCurrentLimit(60);
      m_HaOne.setSmartCurrentLimit(60);
      m_HaTwo.setSmartCurrentLimit(60);
    } else {
      m_McOne.setSmartCurrentLimit(10);
      m_McTwo.setSmartCurrentLimit(10);
      m_HaOne.setSmartCurrentLimit(10);
      m_HaTwo.setSmartCurrentLimit(10);
    }
  }

  public boolean getclimbingenable() {
    return climbEnabble;
  }

  public void midClimb() {
    if (climbEnabble) {
      McjoystickAxis = -m_climbJoystick.getRawAxis(Constants.leftJoystickY);
      if (McjoystickAxis > 0.1 || McjoystickAxis < -0.1) {
        if (McjoystickAxis > 0) {
          if (McHeightOne + (McjoystickAxis / 10 * 8) <= Constants.McHeightMax) {
            McHeightOne = McHeightOne + (McjoystickAxis / 10 * 15);
          }
          if (McHeightTwo + (McjoystickAxis / 10 * 8) <= Constants.McHeightMax) {
            McHeightTwo = McHeightTwo + (McjoystickAxis / 10 * 15);
          }
        }
        if (McjoystickAxis < 0) {
          if (McHeightOne + (McjoystickAxis / 10 * 6) >= 0) {
            McHeightOne = McHeightOne + (McjoystickAxis / 10 * 6);
          }
          if (McHeightTwo + (McjoystickAxis / 10 * 6) >= 0) {
            McHeightTwo = McHeightTwo + (McjoystickAxis / 10 * 6);
          }
        }
      }
      m_McOne.getPIDCtrl().setReference(McHeightOne, CANSparkMax.ControlType.kPosition);
      sbMcHeightOne.setNumber(McHeightOne);

      m_McTwo.getPIDCtrl().setReference(McHeightTwo, CANSparkMax.ControlType.kPosition);
      sbMcHeightTwo.setNumber(McHeightTwo);
    }
  }

  public void highArms() {
    if (climbEnabble) {
      HajoystickAxis = -m_climbJoystick.getRawAxis(Constants.rightJoystickY);
      if (HajoystickAxis > 0.1 || HajoystickAxis < -0.1) {
        if (HajoystickAxis > 0) {
          if (HaHeightOne + (HajoystickAxis / 10 * 8) <= Constants.HaHeightMax) {
            HaHeightOne = HaHeightOne + (HajoystickAxis / 10 * 15);
          }
          if (HaHeightTwo + (HajoystickAxis / 10 * 8) <= Constants.HaHeightMax) {
            HaHeightTwo = HaHeightTwo + (HajoystickAxis / 10 * 15);
          }
        }
        if (HajoystickAxis < 0) {
          if (HaHeightOne + (HajoystickAxis / 10 * 6) >= 0) {
            HaHeightOne = HaHeightOne + (HajoystickAxis / 10 * 6);
          }
          if (HaHeightTwo + (HajoystickAxis / 10 * 6) >= 0) {
            HaHeightTwo = HaHeightTwo + (HajoystickAxis / 10 * 6);
          }
        }
      }
      m_HaOne.getPIDCtrl().setReference(HaHeightOne, CANSparkMax.ControlType.kPosition);
      sbHaHeightOne.setNumber(HaHeightOne);

      m_HaTwo.getPIDCtrl().setReference(HaHeightTwo, CANSparkMax.ControlType.kPosition);
      sbHaHeightTwo.setNumber(HaHeightTwo);
    }
  }

  public void periodic() {
    if (DriverStation.isDisabled() && climbEnabble) {
      climbEnable();
    }
    SmartDashboard.putNumber("Mc1 Applied Output", m_McOne.getAppliedOutput());
    SmartDashboard.putNumber("Mc1 Hight", m_McOne.getEncoder().getPosition());
    SmartDashboard.putNumber("Mc1 IAccum", m_McOne.getPIDCtrl().getIAccum());

    SmartDashboard.putNumber("Mc2 Applied Output", m_McTwo.getAppliedOutput());
    SmartDashboard.putNumber("Mc2 Hight", m_McTwo.getEncoder().getPosition());
    SmartDashboard.putNumber("Mc2 IAccum", m_McTwo.getPIDCtrl().getIAccum());

    SmartDashboard.putNumber("Ha1 Applied Output", m_HaOne.getAppliedOutput());
    SmartDashboard.putNumber("Ha1 Hight", m_HaOne.getEncoder().getPosition());
    SmartDashboard.putNumber("Ha1 IAccum", m_HaOne.getPIDCtrl().getIAccum());

    SmartDashboard.putNumber("Ha2 Applied Output", m_HaTwo.getAppliedOutput());
    SmartDashboard.putNumber("Ha2 Hight", m_HaTwo.getEncoder().getPosition());
    SmartDashboard.putNumber("Ha2 IAccum", m_HaTwo.getPIDCtrl().getIAccum());

    // Mid Climb
    if (sbMcHeightOne.getDouble(0) != McHeightOne) {
      McHeightOne = sbMcHeightOne.getDouble(0);
    } else {
      sbMcTargettedOne.setDouble(McHeightOne);
    }
    sbMcSpeedOne.setDouble(m_McOne.getEncoder().getVelocity());

    if (sbMcHeightTwo.getDouble(0) != McHeightTwo) {
      McHeightTwo = sbMcHeightTwo.getDouble(0);
    } else {
      sbMcTargettedTwo.setDouble(McHeightTwo);
    }
    sbMcSpeedTwo.setDouble(m_McTwo.getEncoder().getVelocity());

    // High Arms
    if (sbHaHeightOne.getDouble(0) != HaHeightOne) {
      HaHeightOne = sbHaHeightOne.getDouble(0);
    } else {
      sbHaTargettedOne.setDouble(HaHeightOne);
    }
    sbHaSpeedOne.setDouble(m_HaOne.getEncoder().getVelocity());

    if (sbHaHeightTwo.getDouble(0) != HaHeightTwo) {
      HaHeightTwo = sbHaHeightTwo.getDouble(0);
    } else {
      sbHaTargettedTwo.setDouble(HaHeightTwo);
    }
    sbHaSpeedTwo.setDouble(m_HaTwo.getEncoder().getVelocity());

    if ((m_McOne.getPIDCtrl().getP() != sbMcOneP.getDouble(0))
        || (m_McOne.getPIDCtrl().getI() != sbMcOneI.getDouble(0))
        || (m_McOne.getPIDCtrl().getD() != sbMcOneD.getDouble(0))
        || (m_HaOne.getPIDCtrl().getP() != sbMcOneP.getDouble(0))
        || (m_HaOne.getPIDCtrl().getI() != sbMcOneI.getDouble(0))
        || (m_HaOne.getPIDCtrl().getD() != sbMcOneD.getDouble(0))) {

      m_McOne.getPIDCtrl().setP(sbMcOneP.getDouble(0));
      m_McOne.getPIDCtrl().setI(sbMcOneI.getDouble(0));
      m_McOne.getPIDCtrl().setD(sbMcOneD.getDouble(0));

      m_HaOne.getPIDCtrl().setP(sbHaOneP.getDouble(0));
      m_HaOne.getPIDCtrl().setI(sbHaOneI.getDouble(0));
      m_HaOne.getPIDCtrl().setD(sbHaOneD.getDouble(0));
    }

    if ((m_McTwo.getPIDCtrl().getP() != sbMcTwoP.getDouble(0))
        || (m_McTwo.getPIDCtrl().getI() != sbMcTwoI.getDouble(0))
        || (m_McTwo.getPIDCtrl().getD() != sbMcTwoD.getDouble(0))
        || (m_HaTwo.getPIDCtrl().getP() != sbMcTwoP.getDouble(0))
        || (m_HaTwo.getPIDCtrl().getI() != sbMcTwoI.getDouble(0))
        || (m_HaTwo.getPIDCtrl().getD() != sbMcTwoD.getDouble(0))) {

      m_McTwo.getPIDCtrl().setP(sbMcTwoP.getDouble(0));
      m_McTwo.getPIDCtrl().setI(sbMcTwoI.getDouble(0));
      m_McTwo.getPIDCtrl().setD(sbMcTwoD.getDouble(0));

      m_HaTwo.getPIDCtrl().setP(sbHaTwoP.getDouble(0));
      m_HaTwo.getPIDCtrl().setI(sbHaTwoI.getDouble(0));
      m_HaTwo.getPIDCtrl().setD(sbHaTwoD.getDouble(0));
    }
    sbClimbEnabble.setBoolean(climbEnabble);
  }

  public void instantiateDebugTab() {
    climbTab = Shuffleboard.getTab("ClimbBase");

    if (Organization) {
      // Shuffle Board Widgets
      climbTab = Shuffleboard.getTab("ClimbBase");

      if (Organization) {
        // Climb Arm 1
        sbMcOneP =
            climbTab
                .add("Mc1 P", Constants.McRightPID[0])
                .withSize(2, 1)
                .withPosition(0, 2)
                .getEntry();
        sbMcOneI =
            climbTab
                .add("Mc1 I", Constants.McRightPID[1])
                .withSize(2, 1)
                .withPosition(0, 3)
                .getEntry();
        sbMcOneD =
            climbTab
                .add("Mc1 D", Constants.McRightPID[2])
                .withSize(2, 1)
                .withPosition(0, 4)
                .getEntry();
        sbMcTargettedOne =
            climbTab.add("Mc1 targetted", 0).withSize(2, 2).withPosition(2, 2).getEntry();
        sbMcHeightOne = climbTab.add("Mc1 Height", 0).withSize(2, 1).withPosition(0, 1).getEntry();
        sbMcSpeedOne = climbTab.add("Mc1 Current", 0).withSize(2, 1).withPosition(2, 4).getEntry();

        // Climb Arm 2
        sbMcTwoP =
            climbTab
                .add("Mc2 P", Constants.McLeftPID[0])
                .withSize(2, 1)
                .withPosition(8, 2)
                .getEntry();
        sbMcTwoI =
            climbTab
                .add("Mc2 I", Constants.McLeftPID[1])
                .withSize(2, 1)
                .withPosition(8, 3)
                .getEntry();
        sbMcTwoD =
            climbTab
                .add("Mc2 D", Constants.McLeftPID[2])
                .withSize(2, 1)
                .withPosition(8, 4)
                .getEntry();
        sbMcTargettedTwo =
            climbTab.add("Mc2 targetted", 0).withSize(2, 2).withPosition(6, 2).getEntry();
        sbMcHeightTwo = climbTab.add("Mc2 Current", 0).withSize(2, 1).withPosition(8, 1).getEntry();
        sbMcSpeedTwo = climbTab.add("Mc2 Speed", 0).withSize(2, 1).withPosition(6, 4).getEntry();
      }

      if (Organization) {
        // High Arm 1
        sbHaOneP =
            climbTab
                .add("Ha1 P", Constants.HaRightPID[0])
                .withSize(2, 1)
                .withPosition(0, 2)
                .getEntry();
        sbHaOneI =
            climbTab
                .add("Ha1 I", Constants.HaRightPID[0])
                .withSize(2, 1)
                .withPosition(0, 2)
                .getEntry();
        sbHaOneD =
            climbTab
                .add("Ha1 D", Constants.HaRightPID[0])
                .withSize(2, 1)
                .withPosition(0, 2)
                .getEntry();
        sbHaTargettedOne =
            climbTab.add("Ha1 targetted", 0).withSize(2, 2).withPosition(6, 2).getEntry();
        sbHaSpeedOne = climbTab.add("Ha1 Speed", 0).withSize(2, 1).withPosition(6, 4).getEntry();
        sbHaHeightOne = climbTab.add("Ha1 Current", 0).withSize(2, 1).withPosition(8, 1).getEntry();

        // High Arm 2
        sbHaTwoP =
            climbTab
                .add("Ha2 P", Constants.HaLeftPID[0])
                .withSize(2, 1)
                .withPosition(0, 2)
                .getEntry();
        sbHaTwoI =
            climbTab
                .add("Ha2 I", Constants.HaLeftPID[0])
                .withSize(2, 1)
                .withPosition(0, 2)
                .getEntry();
        sbHaTwoD =
            climbTab
                .add("Ha2 D", Constants.HaLeftPID[0])
                .withSize(2, 1)
                .withPosition(0, 2)
                .getEntry();
        sbHaTargettedTwo =
            climbTab.add("Ha2 targetted", 0).withSize(2, 2).withPosition(6, 2).getEntry();
        sbMcSpeedTwo = climbTab.add("Ha2 Speed", 0).withSize(2, 1).withPosition(6, 4).getEntry();
        sbHaHeightTwo = climbTab.add("Ha2 Current", 0).withSize(2, 1).withPosition(8, 1).getEntry();
      }

      // Other
      sbClimbEnabble =
          climbTab.add("Climb Enabled", false).withSize(3, 2).withPosition(2, 0).getEntry();
    }
  }

  @Deprecated
  public void runManual() {
    if (climbEnabble) {

      McjoystickAxis = -m_climbJoystick.getRawAxis(Constants.leftJoystickY);
      if (McjoystickAxis > 0.1 || McjoystickAxis < -0.1) {
        if (McjoystickAxis > 0) {
          m_McOne.set(sbClimbSpeedInput.getDouble(0));
          m_McTwo.set(sbClimbSpeedInput.getDouble(0));
        }
        if (McjoystickAxis < 0) {
          m_McOne.set(-sbClimbSpeedInput.getDouble(0));
          m_McTwo.set(-sbClimbSpeedInput.getDouble(0));
        }
      } else {
        m_McOne.set(0);
        m_McTwo.set(0);
      }
    }
  }
}
