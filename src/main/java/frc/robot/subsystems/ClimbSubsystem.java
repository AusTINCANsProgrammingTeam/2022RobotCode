// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ClimbSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Joystick m_climbJoystick;
  private MotorController m_climbMotorControllerOne;
  private MotorController m_climbMotorControllerTwo;
  private DigitalInput m_limitSwitch;
  private boolean toggleClimb;
  private double climbHeight;
  private ShuffleboardTab climberTab = Shuffleboard.getTab("Climber Tab");
  private NetworkTableEntry climblock = climberTab.add("Climb Lock", false).withPosition(0, 0)
      .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  private NetworkTableEntry climberposition = climberTab.add("Climber position", 0).withPosition(1, 0).getEntry();
  private NetworkTableEntry climberspeed = climberTab.add("Climber Current Speed", 0).withPosition(2, 0).getEntry();
  private NetworkTableEntry climberheight = climberTab.add("Climber targetted height", 0).withPosition(3, 0).getEntry();

  public ClimbSubsystem(Joystick joystick) {
    m_climbJoystick = joystick;
    toggleClimb = false;
    climbHeight = 0;

    // One is left, two is right
    m_climbMotorControllerOne = new MotorController("Climb Motor One", Constants.ClimbMotorOne, 40, true);
    m_climbMotorControllerTwo = new MotorController("Climb Motor Two", Constants.ClimbMotorTwo, 40, true);
    m_climbMotorControllerTwo.setInverted(true);
    m_climbMotorControllerTwo.setFollow(m_climbMotorControllerOne);
    m_climbMotorControllerOne.getEncoder().setPosition(0);

    m_limitSwitch = new DigitalInput(Constants.LimitSwitchChannel);
  }

  public void toggleClimbEnable() {
    toggleClimb = !toggleClimb;
  }

  public void enableClimb() {
    if (climblock.getBoolean(false) && !m_limitSwitch.get()) {
      climbHeight = climbHeight + m_climbJoystick.getRawAxis(Constants.leftJoystickY);
      m_climbMotorControllerOne
          .getPID()
          .setReference(climbHeight, CANSparkMax.ControlType.kPosition);
    } else {
      m_climbMotorControllerOne.getPID().setReference(0, CANSparkMax.ControlType.kVoltage);
    }
  }

  public void periodic() {
    SmartDashboard.putNumber("Climb Hight", m_climbMotorControllerOne.getEncoder().getPosition());
    climberheight.setDouble(climbHeight);
    climberspeed.setDouble(m_climbMotorControllerOne.getEncoder().getVelocity());
    climberposition.setDouble(m_climbMotorControllerOne.getEncoder().getPosition());

  }

  public boolean getLimitSwitchVal() {
    return m_limitSwitch.get();
  }
  // TODO: might add other getter methods depending on how many limit switches
}
