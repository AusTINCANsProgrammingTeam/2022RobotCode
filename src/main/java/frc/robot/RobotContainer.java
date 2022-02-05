// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.subsystems.DriveBaseSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.BeamBreakCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.ShooterPrime;
import frc.robot.commands.CDSForwardCommand;
import frc.robot.commands.CDSReverseCommand;

 // This class is where the bulk of the robot should be declared. Since Command-based is a
 // "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 // perieodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 // subsystems, commands, and button mappings) should be declared here. 
 

public class RobotContainer {
  public static ShuffleboardTab debugTab;

  // The robot's subsystems and commands are defined here...
  private static final Joystick driverJoystick = new Joystick(Constants.portNumber);
  private JoystickButton[] buttons = new JoystickButton[11];


  // subsystems
  private static final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem(driverJoystick);
  private final CDSSubsystem CDSSubsystem = new CDSSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(); 
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  // commands
  private final DriveBaseTeleopCommand driveBaseTeleopCommand = new DriveBaseTeleopCommand(driveBaseSubsystem);
  private IntakeForwardCommand intakeForwardCommand = new IntakeForwardCommand(intakeSubsystem);
  private IntakeReverseCommand intakeReverseCommand = new IntakeReverseCommand(intakeSubsystem);
   // private BeamBreakCommand beamBreakCommand = new BeamBreakCommand(intakeSubsystem);
  private ShooterPrime shooterPrime = new ShooterPrime(shooterSubsystem,limelightSubsystem);
  private CDSForwardCommand CDSForwardCommand = new CDSForwardCommand(CDSSubsystem,shooterSubsystem);
  private CDSReverseCommand CDSReverseCommand = new CDSReverseCommand(CDSSubsystem);
  private LimelightAlign limelightAlign = new LimelightAlign(limelightSubsystem,driveBaseSubsystem);

  // auton
  // private Trajectory[] mTrajectories;  // multiple trajectories
  // private int trajectoryIndex = 0;
  private Trajectory trajectory;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    debugTab = Shuffleboard.getTab("debug");
    // Configure the button bindings
    for (int i = 1; i < buttons.length; i++) {
      buttons[i] = new JoystickButton(driverJoystick, i);
    }


    configureButtonBindings();
    
    try {
      initializeTrajectories();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    driveBaseSubsystem.setDefaultCommand(driveBaseTeleopCommand);

//    intakeSubsystem.setDefaultCommand(beamBreakCommand);

  }

  // Use this method to define your button->command mappings. Buttons can be
  // created by
  // instantiating a {@link GenericHID} or one of its subclasses ({@link
  // edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
  // it to a {@link
  // edu.wpi.first.wpilibj2.command.button.JoystickButton}.
  private void configureButtonBindings() {

    // Intake
    buttons[Constants.leftBumperButton].whileHeld(intakeForwardCommand);
    buttons[Constants.rightBumperButton].whileHeld(intakeReverseCommand);
    
    // CDS
    buttons[Constants.leftBumperButton].whileHeld(CDSForwardCommand);
    buttons[Constants.rightBumperButton].whileHeld(CDSReverseCommand);
  
    
    // Shooter
    buttons[Constants.rightTriggerButton].whenPressed(shooterPrime);
    buttons[Constants.upbutton].whenPressed(new InstantCommand(shooterSubsystem::cycleAimModeUp, shooterSubsystem));
    buttons[Constants.downbutton].whenPressed(new InstantCommand(shooterSubsystem::cycleAimModeDown, shooterSubsystem));
    // Limelight
    buttons[Constants.AButton].whenPressed(limelightAlign);
  }

  private void initializeTrajectories() throws IOException {
    // String[] trajectoryJSON = {"One.wpilib.json", "Two.wpilib.json", "Three.wpilib.json", "Four.wpilib.json"};  // add new trajectories manually
    // mTrajectories = new Trajectory[trajectoryJSON.length];
    // for(int i = 0; i < trajectoryJSON.length; i++) {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON[i]);
    //   Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //   mTrajectories[i] = trajectory;
    // }

    // to test auton with just a one straight path
    String trajectoryJSON = "Straight.wpilib.json";
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  }
            
  // Use this to pass the autonomous command to the main {@link Robot} class.
  // @return the command to run in autonomous
  public Command getAutonomousCommand() {
    //Ramsete Command for Pathweaver
    RamseteCommand ramseteCommand =
    new RamseteCommand(
        trajectory,
        driveBaseSubsystem::getPose,
        new RamseteController(Constants.ramseteB, Constants.ramseteZeta), //Fix these constants by
                                                                            //characterizing the robot
        new SimpleMotorFeedforward(
            Constants.sVolts,
            Constants.vVoltSecondsPerMeter,
            Constants.aVoltSecondsSquaredPerMeter),

        Constants.driveKinematics,
        
        driveBaseSubsystem::getWheelSpeeds,
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        //RamseteCommand passes volts to the callback
        driveBaseSubsystem::setAutonVolts,
        driveBaseSubsystem);
        
    driveBaseSubsystem.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand.andThen(() -> driveBaseSubsystem.setAutonVolts(0,0));
  }


  // TODO: create get methods for other subsystems to pass into TabContainer, or find a more efficient way23
  public static DriveBaseSubsystem getDriveBase() {
    return driveBaseSubsystem;
  }
}
