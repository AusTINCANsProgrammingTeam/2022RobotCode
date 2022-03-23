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
  private MotorController m_McMotorControllerOne;
  private MotorController m_McMotorControllerTwo;
  private MotorController m_HaOne;
  private MotorController m_HaTwo;
  private boolean climbEnabbled;
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
  private NetworkTableEntry sbcMcHeightTwo;
  private NetworkTableEntry sbMcTwoP;
  private NetworkTableEntry sbMcTwoI;
  private NetworkTableEntry sbMcTwoD;

  // HA = High Arms
  private NetworkTableEntry sbHaOneCurrent;
  private NetworkTableEntry sbHaSpeedOne;
  private NetworkTableEntry sbHaTargettedHeightOne;
  private NetworkTableEntry sbHaOneP;
  private NetworkTableEntry sbHaOneI;
  private NetworkTableEntry sbHaOneD;

  private NetworkTableEntry sbHaCurrentTwo;
  private NetworkTableEntry sbHaSpeedTwo;
  private NetworkTableEntry sbHaTargettedHeightTwo;
  private NetworkTableEntry sbHaTwoP;
  private NetworkTableEntry sbHaTwoI;
  private NetworkTableEntry sbHaTwoD;

  // Other
  private NetworkTableEntry sbClimbSpeedInput;
  private NetworkTableEntry sbClimbEnabbled;

  private boolean Organization;
  private boolean AmIBeautifull;

  public ClimbSubsystem(Joystick joystick) {
    AmIBeautifull = true;
    m_climbJoystick = joystick;
    Organization = true;
    climbEnabbled = false;
    McHeightOne = 0;
    McHeightTwo = 0;
    HaHeightOne = 0;
    HaHeightTwo = 0;

    // Mid Climb Arms
    // One is left, two is right
    m_McMotorControllerOne =
        new MotorController("Mid Climb Motor One", Constants.McMotorOne, Constants.McLeftPID);
    m_McMotorControllerOne.setSmartCurrentLimit(10);
    m_McMotorControllerTwo =
        new MotorController("Mid Climb Motor Two", Constants.McMotorTwo, Constants.McRightPID);
    m_McMotorControllerTwo.setSmartCurrentLimit(10);
    m_McMotorControllerTwo.setInverted(true);
    m_McMotorControllerOne.getEncoder().setPosition(0);
    m_McMotorControllerTwo.getEncoder().setPosition(0);

    m_McMotorControllerOne.setIdleMode(IdleMode.kBrake);
    m_McMotorControllerTwo.setIdleMode(IdleMode.kBrake);

    // Heigh  Arms
    m_HaOne = new MotorController("Traversal Climb Motor One", Constants.HaMotorOne);
    m_HaOne.setSmartCurrentLimit(10);

    m_HaTwo = new MotorController("Traversal Climb Motor Two", Constants.HaMotorTwo);
    m_HaTwo.setSmartCurrentLimit(10);
    m_HaTwo.setInverted(true);

    if (Organization) {
      // Shuffle Board Widgets
      climbTab = Shuffleboard.getTab("ClimbBase");

      if (Organization) {
      // Climb Arm 1
      climbTab.add("Mc Height 1", 0).withSize(2, 1).withPosition(0, 1).getEntry();
      sbMcOneP =
          climbTab
              .add("Mc One P", Constants.McRightPID[0])
              .withSize(2, 1)
              .withPosition(0, 2)
              .getEntry();
      sbMcOneI =
          climbTab
              .add("Mc One I", Constants.McRightPID[1])
              .withSize(2, 1)
              .withPosition(0, 3)
              .getEntry();
      sbMcOneD =
          climbTab
              .add("climb One D", Constants.McRightPID[2])
              .withSize(2, 1)
              .withPosition(0, 4)
              .getEntry();
      sbMcTargettedOne =
          climbTab.add("Climb targetted height 1", 0).withSize(2, 2).withPosition(2, 2).getEntry();
      sbMcSpeedOne =
          climbTab.add("Climb Current Speed 1", 0).withSize(2, 1).withPosition(2, 4).getEntry();

      // Climb Arm 2
      sbcMcHeightTwo =
          climbTab.add("Climb Height 2", 0).withSize(2, 1).withPosition(8, 1).getEntry();
      sbMcTwoP =
          climbTab
              .add("climb Two P", Constants.McLeftPID[0])
              .withSize(2, 1)
              .withPosition(8, 2)
              .getEntry();
      sbMcTwoI =
          climbTab
              .add("climb Two I", Constants.McLeftPID[1])
              .withSize(2, 1)
              .withPosition(8, 3)
              .getEntry();
      sbMcTwoD =
          climbTab
              .add("climb Two D", Constants.McLeftPID[2])
              .withSize(2, 1)
              .withPosition(8, 4)
              .getEntry();
      sbMcTargettedTwo =
          climbTab.add("Climb targetted height 2", 0).withSize(2, 2).withPosition(6, 2).getEntry();
      sbMcSpeedTwo =
          climbTab.add("climb Current Speed 2", 0).withSize(2, 1).withPosition(6, 4).getEntry();
      }

      if (Organization) {
      // High Arm 1
      sbHaOneP =
          climbTab
              .add("Ha One P", Constants.HaRightPID[0])
              .withSize(2, 1)
              .withPosition(0, 2)
              .getEntry();
      sbHaOneI =
          climbTab
              .add("Ha One I", Constants.HaRightPID[0])
              .withSize(2, 1)
              .withPosition(0, 2)
              .getEntry();
      sbHaOneD =
          climbTab
              .add("Ha One D", Constants.HaRightPID[0])
              .withSize(2, 1)
              .withPosition(0, 2)
              .getEntry();
      sbHaTargettedHeightOne =
          climbTab.add("Ha targetted 1", 0).withSize(2, 2).withPosition(6, 2).getEntry();
      sbMcSpeedOne = climbTab.add("Ha Current 1", 0).withSize(2, 1).withPosition(6, 4).getEntry();

      // High Arm 2
      sbHaOneP =
          climbTab
              .add("Ha Two P", Constants.HaLeftPID[0])
              .withSize(2, 1)
              .withPosition(0, 2)
              .getEntry();
      sbHaOneI =
          climbTab
              .add("Ha Two I", Constants.HaLeftPID[0])
              .withSize(2, 1)
              .withPosition(0, 2)
              .getEntry();
      sbHaOneD =
          climbTab
              .add("Ha Two D", Constants.HaLeftPID[0])
              .withSize(2, 1)
              .withPosition(0, 2)
              .getEntry();
      sbHaTargettedHeightTwo =
          climbTab.add("Ha targetted 2", 0).withSize(2, 2).withPosition(6, 2).getEntry();
      sbMcSpeedTwo = climbTab.add("Ha Current 2", 0).withSize(2, 1).withPosition(6, 4).getEntry();
      }
      
      // Other
      sbClimbEnabbled =
          climbTab.add("Climb Enabled", false).withSize(3, 2).withPosition(2, 0).getEntry();
    }
  }

  public void resetTargetedHeight() {
    McHeightOne = m_McMotorControllerOne.getEncoder().getPosition();
    McHeightTwo = m_McMotorControllerTwo.getEncoder().getPosition();

    HaHeightOne = m_HaOne.getEncoder().getPosition();
    HaHeightTwo = m_HaTwo.getEncoder().getPosition();
  }

  public void climbKeepDownFunction() {
    m_McMotorControllerOne
        .getPIDCtrl()
        .setReference(McHeightOne, CANSparkMax.ControlType.kPosition);
    sbMcHeightOne.setNumber(McHeightOne);

    m_McMotorControllerTwo
        .getPIDCtrl()
        .setReference(McHeightTwo, CANSparkMax.ControlType.kPosition);
    sbcMcHeightTwo.setNumber(McHeightTwo);

    m_HaOne.getPIDCtrl().setReference(HaHeightOne, CANSparkMax.ControlType.kPosition);
    // Insert High Arm Shuffleboard after done

    m_HaTwo.getPIDCtrl().setReference(HaHeightTwo, CANSparkMax.ControlType.kPosition);
  }

  public void climbEnable() {
    climbEnabbled = !climbEnabbled;
    sbClimbEnabbled.setBoolean(climbEnabbled);
    if (climbEnabbled) {
      m_McMotorControllerOne.setSmartCurrentLimit(60);
      m_McMotorControllerTwo.setSmartCurrentLimit(60);
      m_HaOne.setSmartCurrentLimit(60);
      m_HaTwo.setSmartCurrentLimit(60);
    } else {
      m_McMotorControllerOne.setSmartCurrentLimit(10);
      m_McMotorControllerTwo.setSmartCurrentLimit(10);
      m_HaOne.setSmartCurrentLimit(10);
      m_HaTwo.setSmartCurrentLimit(10);
    }
  }

  public boolean getclimbingenable() {
    return sbClimbEnabbled.getBoolean(false);
  }

  public void runManual() {
    if (climbEnabbled) {

      McjoystickAxis = -m_climbJoystick.getRawAxis(Constants.leftJoystickY);
      if (McjoystickAxis > 0.1 || McjoystickAxis < -0.1) {
        if (McjoystickAxis > 0) {
          m_McMotorControllerOne.set(sbClimbSpeedInput.getDouble(0));
          m_McMotorControllerTwo.set(sbClimbSpeedInput.getDouble(0));
        }
        if (McjoystickAxis < 0) {
          m_McMotorControllerOne.set(-sbClimbSpeedInput.getDouble(0));
          m_McMotorControllerTwo.set(-sbClimbSpeedInput.getDouble(0));
        }
      } else {
        m_McMotorControllerOne.set(0);
        m_McMotorControllerTwo.set(0);
      }
    }
  }

  public void enableClimb() {
    if (climbEnabbled) {

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

      m_McMotorControllerOne
          .getPIDCtrl()
          .setReference(McHeightOne, CANSparkMax.ControlType.kPosition);
      sbMcHeightOne.setNumber(McHeightOne);

      m_McMotorControllerTwo
          .getPIDCtrl()
          .setReference(McHeightTwo, CANSparkMax.ControlType.kPosition);
      sbcMcHeightTwo.setNumber(McHeightTwo);
      
      m_HaOne.getPIDCtrl().setReference(HaHeightOne,CANSparkMax.ControlType.kPosition);
      m_HaTwo.getPIDCtrl().setReference(HaHeightTwo, CANSparkMax.ControlType.kPosition);
    } else {
      // m_climbMotorControllerOne.getPID().setReference(0,
      // CANSparkMax.ControlType.kVoltage);
      //m_HaOne.getPIDCtrl().setReference(0,CANSparkMax.ControlType.kPosition);
      //m_HaTwo.getPIDCtrl().setReference(0, CANSparkMax.ControlType.kPosition);
    }
  }

  public void periodic() {
    if (DriverStation.isDisabled() && climbEnabbled) {
      climbEnable();
    }

    SmartDashboard.putNumber(
        "MC Climb motor 1 Applied Output", m_McMotorControllerOne.getAppliedOutput());
    SmartDashboard.putNumber(
        "MC Climb motor 2 Applied Output", m_McMotorControllerTwo.getAppliedOutput());
    SmartDashboard.putNumber(
      "HA Climb motor 1 Applied Output", m_HaOne.getAppliedOutput());
    SmartDashboard.putNumber(
        "HA Climb motor 2 Applied Output", m_HaTwo.getAppliedOutput());
    SmartDashboard.putNumber("Climb Hight One", m_McMotorControllerOne.getEncoder().getPosition());
    if (sbMcHeightOne.getDouble(0) != McHeightOne) {
      McHeightOne = sbMcHeightOne.getDouble(0);
    } else {
      sbMcTargettedOne.setDouble(McHeightOne);
    }
    sbMcSpeedOne.setDouble(m_McMotorControllerOne.getEncoder().getVelocity());

    // m_climbMotorControllerOne.updateSmartDashboard();
    SmartDashboard.putNumber("Climb IAccum One", m_McMotorControllerOne.getPIDCtrl().getIAccum());

    SmartDashboard.putNumber("Climb Hight Two", m_McMotorControllerTwo.getEncoder().getPosition());
    if (sbcMcHeightTwo.getDouble(0) != McHeightTwo) {
      McHeightTwo = sbcMcHeightTwo.getDouble(0);
    } else {
      sbMcTargettedTwo.setDouble(McHeightTwo);
    }
    sbMcSpeedTwo.setDouble(m_McMotorControllerTwo.getEncoder().getVelocity());

    // m_climbMotorControllerTwo.updateSmartDashboard();
    SmartDashboard.putNumber("Climb IAccum Two", m_McMotorControllerTwo.getPIDCtrl().getIAccum());
    if ((m_McMotorControllerOne.getPIDCtrl().getP() != sbMcOneP.getDouble(0))
        || (m_McMotorControllerOne.getPIDCtrl().getI() != sbMcOneI.getDouble(0))
        || (m_McMotorControllerOne.getPIDCtrl().getD() != sbMcOneD.getDouble(0)) 
        || (m_HaOne.getPIDCtrl().getP() != sbMcOneD.getDouble(0))
        || (m_HaOne.getPIDCtrl().getI() != sbMcOneD.getDouble(0))
        || (m_HaOne.getPIDCtrl().getD() != sbMcOneD.getDouble(0))){
      m_McMotorControllerOne.getPIDCtrl().setP(sbMcOneP.getDouble(0));
      m_McMotorControllerOne.getPIDCtrl().setI(sbMcOneI.getDouble(0));
      m_McMotorControllerOne.getPIDCtrl().setD(sbMcOneD.getDouble(0));
      m_HaOne
    }

    if ((m_McMotorControllerTwo.getPIDCtrl().getP() != sbMcTwoP.getDouble(0))
        || (m_McMotorControllerTwo.getPIDCtrl().getI() != sbMcTwoI.getDouble(0))
        || (m_McMotorControllerTwo.getPIDCtrl().getD() != sbMcTwoD.getDouble(0))) {
      m_McMotorControllerOne.getPIDCtrl().setP(sbMcOneP.getDouble(0));
      m_McMotorControllerOne.getPIDCtrl().setI(sbMcOneI.getDouble(0));
      m_McMotorControllerOne.getPIDCtrl().setD(sbMcOneD.getDouble(0));
    }
    
  }
}
