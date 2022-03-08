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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonModes;
import frc.robot.commands.CDSBallManagementCommand;
import frc.robot.commands.CDSForwardCommand;
import frc.robot.commands.ClimbCommand;
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
  private static CDSSubsystem CDSSubsystem;
  private static IntakeSubsystem intakeSubsystem;
  private static ShooterSubsystem shooterSubsystem;
  private static LimelightSubsystem limelightSubsystem;

  // commands
  private DriveBaseTeleopCommand driveBaseTeleopCommand;
  private IntakeForwardCommand intakeForwardCommand;
  private IntakeReverseCommand intakeReverseCommand;
  private ClimbCommand climbCommand;

  private ShooterHeld shooterHeldLow, shooterHeldAuto;
  private CDSForwardCommand CDSForwardCommand;
  private OuttakeCommand outtakeCommand;
  private LimelightAlign limelightAlign;

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

    initAuton();
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
    for (Constants.Subsystems sub : Constants.Subsystems.values()) {
      if (sub.isEnabled()) {

        controllerCheck();

        // System.out.println((String) k + " " + subSysEnables.get((String) k));
        switch (sub.toString()) {
          case "DriveBaseSubsystem":
            {
              System.out.println("Drivebase enabled");
              driveBaseSubsystem = new DriveBaseSubsystem(driverJoystick, Constants.usingExternal);
              break;
            }
          case "CDSSubsystem":
            {
              System.out.println("CDS enabled");
              CDSSubsystem = new CDSSubsystem();
              break;
            }
          case "IntakeSubsystem":
            {
              System.out.println("Intake enabled");
              intakeSubsystem = new IntakeSubsystem();
              break;
            }
          case "ShooterSubsystem":
            {
              System.out.println("Shooter enabled");
              shooterSubsystem = new ShooterSubsystem();
              break;
            }
          case "LimelightSubsystem":
            {
              System.out.println("Limelight enabled");
              limelightSubsystem = new LimelightSubsystem();
              break;
            }
          case "ClimbSubsystem":
            {
              if (axisCount1 > 0 && buttonCount1 > 0) {
                climbSubsystem = new ClimbSubsystem(operatorJoystick);
                climbCommand = new ClimbCommand(climbSubsystem);
                System.out.println("Climb enabled");
              }
              break;
            }
        }
      }
    }
  }

  private void initCommands() {
    // Initializes commands based on enabled subsystems
    if (driveBaseSubsystem != null) {
      driveBaseTeleopCommand = new DriveBaseTeleopCommand(driveBaseSubsystem);
      driveBaseSubsystem.setDefaultCommand(driveBaseTeleopCommand);
    }
    if (CDSSubsystem != null && shooterSubsystem != null) {
      CDSForwardCommand = new CDSForwardCommand(CDSSubsystem);
    }
    // CDS
    if (CDSSubsystem != null) {
      CDSForwardCommand = new CDSForwardCommand(CDSSubsystem);
      // CDSSubsystem.setDefaultCommand(new CDSAutoAdvanceCommand(CDSSubsystem));
      // CDSReverseCommand = new CDSReverseCommand(CDSSubsystem, shooterSubsystem);
      // CDSSubsystem.senseColor();
    }

    if (intakeSubsystem != null && CDSSubsystem != null) {
      intakeForwardCommand = new IntakeForwardCommand(intakeSubsystem);
      intakeReverseCommand = new IntakeReverseCommand(intakeSubsystem, CDSSubsystem);
      outtakeCommand = new OuttakeCommand(intakeSubsystem, CDSSubsystem);
      //CDSSubsystem.setDefaultCommand(new CDSBallManagementCommand(CDSSubsystem, intakeSubsystem));
    }
    if (shooterSubsystem != null && CDSSubsystem != null) {
      shooterHeldAuto =
          new ShooterHeld(
              shooterSubsystem, limelightSubsystem, CDSSubsystem, (limelightSubsystem != null));
      shooterHeldLow =
          new ShooterHeld(
              shooterSubsystem, limelightSubsystem, CDSSubsystem, (limelightSubsystem != null));
    }
    if (limelightSubsystem != null && driveBaseSubsystem != null) {
      limelightAlign = new LimelightAlign(limelightSubsystem, driveBaseSubsystem);
    }
    if (climbSubsystem != null) {
      climbSubsystem.setDefaultCommand(climbCommand);
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
    if (intakeForwardCommand != null && outtakeCommand != null) {
      // takes ball in
      buttons[Constants.RTriggerButton].whileHeld(intakeForwardCommand);
      // spits ball out
      buttons[Constants.RBumper].whileHeld(outtakeCommand);
    }

    if (shooterSubsystem != null && shooterHeldLow != null && shooterHeldAuto != null) {
      // Auto Aim Shot
      buttons[Constants.LTriggerButton].whileHeld(
          shooterHeldAuto.beforeStarting(
              () -> {
                shooterSubsystem.setAimMode(Constants.AimModes.TEST);
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

    if (axisCount1 == 0 && buttonCount1 == 0) {

      // Shooter
      if (shooterSubsystem != null && shooterHeldAuto != null) {
        buttons[Constants.backButton].whenPressed(shooterHeldAuto);
        buttons[Constants.LJoystickButton].whenPressed(
            new InstantCommand(shooterSubsystem::cycleAimModeNext, shooterSubsystem));
        buttons[Constants.RJoystickButton].whenPressed(
            new InstantCommand(shooterSubsystem::cycleAimModePrevious, shooterSubsystem));
      }

      // Limelight
      if (limelightAlign != null) {
        buttons[Constants.startButton].whenPressed(limelightAlign);
      }

      // ClimbSubysystem has no binding because there are not enuf axises
      if (climbSubsystem != null) {}

      System.out.printf("Using Testing One-controller button mappings");
    } else {

      if (climbSubsystem != null) {
        buttons2[Constants.startButton].whenPressed(
            new InstantCommand(climbSubsystem::climbEnabbledEnable, climbSubsystem));
      }

      if (outtakeCommand != null && CDSForwardCommand != null) {
        buttons[Constants.RTriggerButton].whileHeld(CDSForwardCommand);
        buttons2[Constants.RBumper].whileHeld(outtakeCommand);
      }

      System.out.printf("Using Competition Two-controller button mappings");
    }
  }

  public Command getAutonomousCommand(String pathname) {
    if (autonModes != null) {
      chosenAutonMode = autonModes.getChosenCommand(pathname);
      return chosenAutonMode;
    }
    return null;
  }

  private void initAuton() {
    if (driveBaseSubsystem != null) {
      if (shooterSubsystem != null
          && limelightSubsystem != null
          && CDSSubsystem != null
          && intakeSubsystem != null) {
        autonModes =
            new AutonModes(
                driveBaseSubsystem,
                shooterSubsystem,
                limelightSubsystem,
                CDSSubsystem,
                intakeSubsystem);

      } else {
        autonModes =
            new AutonModes(
                driveBaseSubsystem); // default constructor, if other subsystems are disabled only
        // use drivebase for taxi
      }
    } else {
      autonModes = null;
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
