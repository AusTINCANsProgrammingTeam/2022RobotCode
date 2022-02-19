// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.subsystems.DriveBaseSubsystem;

=======
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
>>>>>>> origin/main
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.ShooterPrime;
=======
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
>>>>>>> origin/main
import frc.robot.commands.CDSForwardCommand;
import frc.robot.commands.CDSReverseCommand;
import frc.robot.commands.ClimbDOWNCommand;
import frc.robot.commands.ClimbUPCommand;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.ShooterHeld;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.io.IOException;
import java.nio.file.Path;

<<<<<<< HEAD
import frc.robot.commands.AutonModes;

 // This class is where the bulk of the robot should be declared. Since Command-based is a
 // "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 // perieodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 // subsystems, commands, and button mappings) should be declared here. 
 
=======
// This class is where the bulk of the robot should be declared. Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
// perieodic methods (other than the scheduler calls). Instead, the structure of the robot
// (including
// subsystems, commands, and button mappings) should be declared here.
>>>>>>> origin/main

public class RobotContainer {
  public static ShuffleboardTab debugTab;

  // The robot's subsystems and commands are defined here...
  private static final Joystick driverJoystick = new Joystick(Constants.portNumber0);
  private JoystickButton[] buttons = new JoystickButton[13];

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
  private ClimbUPCommand climbUPCommand;
  private ClimbDOWNCommand climbDOWNCommand;

  // private BeamBreakCommand beamBreakCommand = new BeamBreakCommand(intakeSubsystem);
  private ShooterHeld shooterHeld;
  private CDSForwardCommand CDSForwardCommand;
  private CDSReverseCommand CDSReverseCommand;
  private LimelightAlign limelightAlign;

<<<<<<< HEAD
  //auton
  private AutonModes autonModes;
  private Command chosenAutonMode;
=======
  // auton
  private Trajectory[] mTrajectories; // multiple trajectories
  private int trajectoryIndex = 0;
  private Trajectory trajectory;
>>>>>>> origin/main

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    debugTab = Shuffleboard.getTab("debug");

    initSubsystems();
    initCommands();

    // initialize the button bindings
    for (int i = 1; i < buttons.length; i++) {
      buttons[i] = new JoystickButton(driverJoystick, i);
    }
    configureButtonBindings();
<<<<<<< HEAD
    
    initAuton();   
=======

    initializeTrajectories();
>>>>>>> origin/main
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
              driveBaseSubsystem = new DriveBaseSubsystem(driverJoystick, false);
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
              System.out.println("Climb enabled");
              climbSubsystem = new ClimbSubsystem();
              climbUPCommand = new ClimbUPCommand(climbSubsystem);
              climbDOWNCommand = new ClimbDOWNCommand(climbSubsystem);
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
      CDSReverseCommand = new CDSReverseCommand(CDSSubsystem);
    }
    if (intakeSubsystem != null) {
      intakeForwardCommand = new IntakeForwardCommand(intakeSubsystem);
      intakeReverseCommand = new IntakeReverseCommand(intakeSubsystem);
    }
    if (shooterSubsystem != null && CDSSubsystem != null) {
      shooterHeld =
          new ShooterHeld(
              shooterSubsystem, limelightSubsystem, CDSSubsystem, (limelightSubsystem != null));
      shooterHeld = new ShooterHeld(shooterSubsystem, limelightSubsystem, CDSSubsystem, true);
    }
    if (limelightSubsystem != null && driveBaseSubsystem != null) {
      limelightAlign = new LimelightAlign(limelightSubsystem, driveBaseSubsystem);
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
      buttons[Constants.LBumper].whileHeld(intakeForwardCommand);
      buttons[Constants.RBumper].whileHeld(intakeReverseCommand);
    }

    // Shooter
    if (shooterSubsystem != null && shooterHeld != null) {
      buttons[Constants.backButton].whenPressed(shooterHeld);
      buttons[Constants.LJoystickButton].whenPressed(
          new InstantCommand(shooterSubsystem::cycleAimModeNext, shooterSubsystem));
      buttons[Constants.RJoystickButton].whenPressed(
          new InstantCommand(shooterSubsystem::cycleAimModePrevious, shooterSubsystem));
    }

<<<<<<< HEAD
    // CDS
    if (CDSSubsystem != null && shooterSubsystem != null) {
      CDSForwardCommand = new CDSForwardCommand(CDSSubsystem, shooterSubsystem);
      CDSReverseCommand = new CDSReverseCommand(CDSSubsystem, shooterSubsystem);
      CDSSubsystem.senseColor();
    }
    
=======
>>>>>>> origin/main
    if (CDSForwardCommand != null && CDSReverseCommand != null) {
      buttons[Constants.LTriggerButton].whileHeld(CDSForwardCommand);
      buttons[Constants.RTriggerButton].whileHeld(CDSReverseCommand);
      // CDSSubsystem.getAllianceColor();
      CDSSubsystem.senseColor();
    }

    // Limelight
    if (limelightAlign != null) {
      buttons[Constants.startButton].whenPressed(limelightAlign);
    }

<<<<<<< HEAD
    // Climb
=======
>>>>>>> origin/main
    if (climbSubsystem != null) {
      buttons[Constants.AButton].whileHeld(climbUPCommand);
      buttons[Constants.XButton].whileHeld(climbDOWNCommand);
    }
<<<<<<< HEAD
  }

  public Command getAutonomousCommand() {
    return chosenAutonMode;
  }

  private void initAuton() {
    autonModes = new AutonModes(driveBaseSubsystem, shooterSubsystem, limelightSubsystem, CDSSubsystem, intakeSubsystem);

    // TODO: add a sendable chooser
    String commandName = "taxi";
    chosenAutonMode = autonModes.getChosenCommand(commandName);
=======
  }

  private void initializeTrajectories() {
    // auton with just a one straight path
    String trajectoryJSON = "paths/Straight.wpilib.json";
    try {
      Path trajectoryPath =
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve(trajectoryJSON); // goes to scr/main/deploy/paths
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  // Use this to pass the autonomous command to the main {@link Robot} class.
  // @return the command to run in autonomous
  public Command getAutonomousCommand() {
    if (driveBaseSubsystem != null && trajectory != null) {
      // Ramsete Command for Pathweaver
      RamseteCommand ramseteCommand =
          new RamseteCommand(
              trajectory,
              driveBaseSubsystem::getPose,
              new RamseteController(
                  Constants.ramseteB,
                  Constants.ramseteZeta), // ramsete follower to follow trajectory
              Constants.driveKinematics,
              driveBaseSubsystem::acceptWheelSpeeds,
              driveBaseSubsystem);

      driveBaseSubsystem.resetOdometry(trajectory.getInitialPose());

      return ramseteCommand.andThen(() -> driveBaseSubsystem.acceptWheelSpeeds(0, 0));
    }

    return null;
>>>>>>> origin/main
  }

  // TODO: create get methods for other subsystems to pass into TabContainer, or find a more
  // efficient way
  public static DriveBaseSubsystem getDriveBase() {
    if (driveBaseSubsystem == null) {
      return null;
    }
    return driveBaseSubsystem;
  }
}
