// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonModes;
import frc.robot.commands.CDSBallManagementCommand;
import frc.robot.commands.CDSForwardCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbEnable;
import frc.robot.commands.ClimbKeepDown;
import frc.robot.commands.CombinedIntakeCDSForwardCommand;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShooterHeld;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// This class is where the bulk of the robot should be declared. Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
// perieodic methods (other than the scheduler calls). Instead, the structure of the robot
// (including
// subsystems, commands, and button mappings) should be declared here.

public class RobotContainer {
  public static ShuffleboardTab debugTab;
  public static ShuffleboardTab controllerDetection;

  // The robot's subsystems and commands are defined here...
  private static final Joystick driverJoystick = new Joystick(Constants.portNumber0);
  private static final Joystick operatorJoystick = new Joystick(Constants.portNumber1);
  private JoystickButton[] buttons = new JoystickButton[13];
  private JoystickButton[] buttons2 = new JoystickButton[13];

  // subsystems
  private static ClimbSubsystem climbSubsystem;
  private static DriveBaseSubsystem driveBaseSubsystem;
  private static CDSSubsystem cdsSubsystem;
  private static IntakeSubsystem intakeSubsystem;
  private static ShooterSubsystem shooterSubsystem;
  private static LimelightSubsystem limelightSubsystem;

  // commands
  private DriveBaseTeleopCommand driveBaseTeleopCommand;
  private IntakeForwardCommand intakeForwardCommand;
  private IntakeReverseCommand intakeReverseCommand;
  private CDSBallManagementCommand ballManagementCommand;
  private CombinedIntakeCDSForwardCommand combinedIntakeCDS;
  private ClimbCommand climbCommand;

  private ShooterHeld shooterHeldLow, shooterHeldAuto;
  private CDSForwardCommand CDSForwardCommand;
  private OuttakeCommand outtakeCommand;
  private LimelightAlign limelightAlign;
  private ClimbEnable climbEnabling;
  private ClimbKeepDown climbKeepDown;

  // auton
  private AutonModes autonModes;
  private Command chosenAutonMode = null;

  // Controller Check Variables
  private NetworkTableEntry sbaxisCount0;
  private NetworkTableEntry sbaxisCount1;
  private NetworkTableEntry sbbuttonCount0;
  private NetworkTableEntry sbbuttonCount1;
  private int axisCount0;
  private int buttonCount0;
  private int axisCount1;
  private int buttonCount1;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    controllerDetection = Shuffleboard.getTab("Controller Detector");

    sbaxisCount0 = controllerDetection.add("Port0 AxisCount", 0).withSize(2, 2).getEntry();
    sbbuttonCount0 = controllerDetection.add("Port0 ButtonCount", 0).withSize(2, 2).getEntry();
    sbaxisCount1 = controllerDetection.add("Port1 AxisCount", 0).withSize(2, 2).getEntry();
    sbbuttonCount1 = controllerDetection.add("Port1 ButtonCount", 0).withSize(2, 2).getEntry();

    debugTab = Shuffleboard.getTab("debug");

    initSubsystems();
    initCommands();

    // initialize the button bindings
    for (int i = 1; i < buttons.length; i++) {
      buttons[i] = new JoystickButton(driverJoystick, i);
      buttons2[i] = new JoystickButton(operatorJoystick, i);
    }
    configureButtonBindings();
  }

  private void controllerCheck() {
    axisCount0 = DriverStation.getStickAxisCount(Constants.portNumber0);
    buttonCount0 = DriverStation.getStickButtonCount(Constants.portNumber0);
    sbaxisCount0.setDouble(axisCount0);
    sbbuttonCount0.setDouble(buttonCount0);

    axisCount1 = DriverStation.getStickAxisCount(Constants.portNumber1);
    buttonCount1 = DriverStation.getStickButtonCount(Constants.portNumber1);
    sbaxisCount1.setDouble(axisCount1);
    sbbuttonCount1.setDouble(buttonCount1);

    System.out.printf(
        "axisCount0 %d buttonCount0 %d axisCount1 %d buttonCount1 %d\n ",
        axisCount0, buttonCount0, axisCount1, buttonCount1);
  }

  private void initSubsystems() {
    // subsystems
    controllerCheck();

    driveBaseSubsystem = new DriveBaseSubsystem(driverJoystick, Constants.usingExternal);

    cdsSubsystem = new CDSSubsystem();

    intakeSubsystem = new IntakeSubsystem();

    shooterSubsystem = new ShooterSubsystem();

    limelightSubsystem = new LimelightSubsystem();

    climbSubsystem = new ClimbSubsystem(operatorJoystick);
    climbCommand = new ClimbCommand(climbSubsystem);
  }

  private void initCommands() {
    // Initializes commands based on enabled subsystems
    if (driveBaseSubsystem != null) {
      driveBaseTeleopCommand = new DriveBaseTeleopCommand(driveBaseSubsystem);
      driveBaseSubsystem.setDefaultCommand(driveBaseTeleopCommand);
    }
    if (cdsSubsystem != null && shooterSubsystem != null) {
      CDSForwardCommand = new CDSForwardCommand(cdsSubsystem, shooterSubsystem);
    }
    if (intakeSubsystem != null && cdsSubsystem != null) {
      intakeForwardCommand = new IntakeForwardCommand(intakeSubsystem, cdsSubsystem);
      intakeReverseCommand = new IntakeReverseCommand(intakeSubsystem, cdsSubsystem);
      outtakeCommand = new OuttakeCommand(intakeSubsystem, cdsSubsystem);

      if (Constants.ballManagementEnabled) {
        intakeForwardCommand = new IntakeForwardCommand(intakeSubsystem, cdsSubsystem);
        ballManagementCommand = new CDSBallManagementCommand(cdsSubsystem, intakeSubsystem);
        cdsSubsystem.setDefaultCommand(ballManagementCommand);
        combinedIntakeCDS =
        new CombinedIntakeCDSForwardCommand(intakeSubsystem, cdsSubsystem, shooterSubsystem);
      }
    }

    if (shooterSubsystem != null && cdsSubsystem != null) {
      shooterHeldAuto =
          new ShooterHeld(
              shooterSubsystem, limelightSubsystem, cdsSubsystem, (limelightSubsystem != null));
      shooterHeldLow =
          new ShooterHeld(
              shooterSubsystem, limelightSubsystem, cdsSubsystem, (limelightSubsystem != null));
    }
    if (limelightSubsystem != null && driveBaseSubsystem != null) {
      limelightAlign = new LimelightAlign(limelightSubsystem, driveBaseSubsystem);
    }

    if ((climbSubsystem != null) && (driveBaseSubsystem != null)) {
      climbEnabling = new ClimbEnable(climbSubsystem, driveBaseSubsystem);
      climbKeepDown = new ClimbKeepDown(climbSubsystem);
      climbSubsystem.setDefaultCommand(climbKeepDown);
    }
  }

  // Use this method to define your button->command mappings. Buttons can be
  // created by
  // instantiating a {@link GenericHID} or one of its subclasses ({@link
  // edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
  // it to a {@link
  // edu.wpi.first.wpilibj2.command.button.JoystickButton}.
  private void configureButtonBindings() {
    controllerCheck();

    // Intake / CDS
    if (outtakeCommand != null) {
      // spits ball out
      buttons[Constants.AButton].whileHeld(outtakeCommand);
    }

    if (combinedIntakeCDS != null) {
      buttons[Constants.RTriggerButton].whileHeld(combinedIntakeCDS);
    } /*else {
        buttons[Constants.RTriggerButton].whileHeld(intakeForwardCommand);
      }*/

    if (shooterSubsystem != null && shooterHeldLow != null && shooterHeldAuto != null) {
      // Auto Aim Shot
      buttons[Constants.LTriggerButton].whileHeld(
          shooterHeldAuto.beforeStarting(
              () -> {
                shooterSubsystem.setAimMode(Constants.AimModes.TARMAC);
              },
              shooterSubsystem));
      // Fender Shot
      buttons[Constants.LBumper].whileHeld(
          shooterHeldLow.beforeStarting(
              () -> {
                shooterSubsystem.setAimMode(Constants.AimModes.LOW);
              },
              shooterSubsystem));
    }

    if (climbSubsystem != null) {
      buttons2[Constants.startButton].whenPressed(climbEnabling);
    }

    if (outtakeCommand != null && intakeForwardCommand != null) {
      buttons2[Constants.RTriggerButton].whileHeld(intakeForwardCommand);
      buttons2[Constants.RBumper].whileHeld(outtakeCommand);
    }
  }

  public Command getAutonomousCommand(Constants.Auton a) {
    if (autonModes != null) {
      chosenAutonMode = autonModes.getChosenCommand(a);
      return chosenAutonMode;
    }
    return null;
  }

  public void initAuton(Constants.Auton mode) {
    // switch for readibility
    boolean success = true;
    switch (mode) {
      case TEST:
        if (driveBaseSubsystem != null) {
          autonModes = new AutonModes(driveBaseSubsystem);
        } else {
          success = false;
        }
        break;
      case TAXI:
        if (driveBaseSubsystem != null && intakeSubsystem != null && cdsSubsystem != null) {
          autonModes = new AutonModes(driveBaseSubsystem, intakeSubsystem, cdsSubsystem);
        } else {
          success = false;
        }
        break;
      case ONEBALL: // all of these modes go down to the FIVEBALL case
      case TWOBALL:
      case THREEBALL:
      case FOURBALL:
      case FIVEBALL:
        if (shooterSubsystem != null
            && driveBaseSubsystem != null
            && intakeSubsystem != null
            && cdsSubsystem != null) {
          autonModes =
              new AutonModes(
                  driveBaseSubsystem,
                  shooterSubsystem,
                  limelightSubsystem,
                  cdsSubsystem,
                  intakeSubsystem);
        } else {
          success = false;
        }
        break;
      default:
        System.out.println("No mode selected");
        break;
    }
    if (success == false) {
      System.out.println(mode.getName() + " mode unable to be created.");
    }
  }

  // TODO: create get methods for other subsystems to pass into TabContainer, or
  // find a more
  // efficient way
  public static DriveBaseSubsystem getDriveBase() {
    if (driveBaseSubsystem != null) {
      return driveBaseSubsystem;
    }
    return null;
  }
}
