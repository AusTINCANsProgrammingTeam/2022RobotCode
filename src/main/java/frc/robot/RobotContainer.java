// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonModes;
import frc.robot.commands.CDSAutoAdvanceCommand;
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

  // private BeamBreakCommand beamBreakCommand = new BeamBreakCommand(intakeSubsystem);
  private ShooterHeld shooterHeld;
  private CDSForwardCommand CDSForwardCommand;
  private OuttakeCommand outtakeCommand;
  private LimelightAlign limelightAlign;

  // auton
  private AutonModes autonModes;
  private Command chosenAutonMode = null;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    debugTab = Shuffleboard.getTab("debug");

    initSubsystems();
    initCommands();

    // initialize the button bindings
    for (int i = 1; i < buttons.length; i++) {
      buttons[i] = new JoystickButton(driverJoystick, i);
      // buttons2[i] = new JoystickButton(operatorJoystick, i);
    }
    configureButtonBindings();

    initAuton();
  }

  private void initSubsystems() {
    // subsystems
    for (Constants.Subsystems sub : Constants.Subsystems.values()) {
      if (sub.isEnabled()) {

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
              if (!Constants.oneController) {
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
      CDSSubsystem.setDefaultCommand(new CDSAutoAdvanceCommand(CDSSubsystem));
      // CDSReverseCommand = new CDSReverseCommand(CDSSubsystem, shooterSubsystem);
      // CDSSubsystem.senseColor();
    }
    if (intakeSubsystem != null) {
      intakeForwardCommand = new IntakeForwardCommand(intakeSubsystem);
      intakeReverseCommand = new IntakeReverseCommand(intakeSubsystem);
    }
    if (shooterSubsystem != null && CDSSubsystem != null) {
      shooterHeld =
          new ShooterHeld(
              shooterSubsystem, limelightSubsystem, CDSSubsystem, (limelightSubsystem != null));
      shooterHeld = new ShooterHeld(shooterSubsystem, limelightSubsystem, CDSSubsystem, false);
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

    // Intake
    if (intakeForwardCommand != null && intakeReverseCommand != null) {
      buttons[Constants.BButton].whileHeld(intakeForwardCommand);
      buttons[Constants.RTriggerButton].whileHeld(intakeReverseCommand);
    }

    // Shooter
    if (shooterSubsystem != null && shooterHeld != null) {
      buttons[Constants.LTriggerButton].whileHeld(shooterHeld);
      // buttons[Constants.LJoystickButton].whenPressed(
      //   new InstantCommand(shooterSubsystem::cycleAimModeNext, shooterSubsystem));
      // buttons[Constants.RJoystickButton].whenPressed(
      // new InstantCommand(shooterSubsystem::cycleAimModePrevious, shooterSubsystem));
    }

    if (CDSForwardCommand != null && outtakeCommand != null) {
      buttons[Constants.LTriggerButton].whileHeld(CDSForwardCommand);
      // buttons[Constants.RTriggerButton].whileHeld(CDSReverseCommand);
      buttons[Constants.RTriggerButton].whileHeld(outtakeCommand);
    }

    // Limelight
    if (limelightAlign != null) {
      buttons[Constants.startButton].whenPressed(limelightAlign);
    }

    // Climb
    if (climbSubsystem != null) {
      if (!Constants.oneController) {
        buttons2[Constants.startButton].whenPressed(
            new InstantCommand(climbSubsystem::toggleClimbEnable, climbSubsystem));
      }
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

  // TODO: create get methods for other subsystems to pass into TabContainer, or find a more
  // efficient way
  public static DriveBaseSubsystem getDriveBase() {
    if (driveBaseSubsystem != null) {
      return driveBaseSubsystem;
    }
    return null;
  }
}
