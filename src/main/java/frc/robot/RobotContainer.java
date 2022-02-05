// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.subsystems.DriveBaseSubsystem;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
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
  private JoystickButton[] mButtons = new JoystickButton[11];

  // subsystems
  private static DriveBaseSubsystem driveBaseSubsystem;
  private static CDSSubsystem CDSSubsystem;
  private static IntakeSubsystem IntakeSubsystem; 
  private static ShooterSubsystem ShooterSubsystem;
  private static LimelightSubsystem LimelightSubsystem;

  // commands
  private DriveBaseTeleopCommand mDriveBaseTeleopCommand;
  private IntakeForwardCommand intakeForwardCommand;
  private IntakeReverseCommand intakeReverseCommand;
  private ShooterPrime shooterPrime;
  private CDSForwardCommand CDSForwardCommand;
  private CDSReverseCommand CDSReverseCommand;
  private LimelightAlign limelightAlign;


  // auton
  // private Trajectory[] mTrajectories;  // multiple trajectories
  // private int trajectoryIndex = 0;
  private Trajectory trajectory;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    debugTab = Shuffleboard.getTab("debug");

    // Configure the button bindings
    for (int i = 1; i < mButtons.length; i++) {
      mButtons[i] = new JoystickButton(driverJoystick, i);
    }

    // subsystems
    for (Constants.Subsystems sub : Constants.Subsystems.values()) {
      if (sub.isEnabled()) {

        //System.out.println((String) k + " " + subSysEnables.get((String) k));
        switch (sub.toString()) {
          case "DriveBaseSubsystem": 
          {
            System.out.println("Drivebase enabled");
            driveBaseSubsystem = new DriveBaseSubsystem(driverJoystick);
            mDriveBaseTeleopCommand = new DriveBaseTeleopCommand(driveBaseSubsystem);
            driveBaseSubsystem.setDefaultCommand(mDriveBaseTeleopCommand);
            break;
          }
          case "CDSSubsystem": 
          {
            System.out.println("CDS enabled");
            CDSSubsystem = new CDSSubsystem();
            CDSForwardCommand = new CDSForwardCommand(CDSSubsystem);
            CDSReverseCommand = new CDSReverseCommand(CDSSubsystem);
            break;
          }
          case "IntakeSubsystem":
          {
            System.out.println("Intake enabled");
            IntakeSubsystem = new IntakeSubsystem(); 
            intakeForwardCommand = new IntakeForwardCommand(IntakeSubsystem);
            intakeReverseCommand = new IntakeReverseCommand(IntakeSubsystem);
            break;
          }
          case "ShooterSubsystem":
          {
            System.out.println("Shooter enabled");
            ShooterSubsystem = new ShooterSubsystem();
            shooterPrime = new ShooterPrime(ShooterSubsystem, LimelightSubsystem);
            break;
          }
          case "LimelightSubsystem":
          {
            System.out.println("Limelight enabled");
            LimelightSubsystem = new LimelightSubsystem();
            limelightAlign = new LimelightAlign(LimelightSubsystem, driveBaseSubsystem);
            break;
          }

        }
      }
    }

    // commands
    

    configureButtonBindings();
    
    try {
      initializeTrajectories();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
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
    if(IntakeSubsystem != null) {
      mButtons[Constants.leftBumperButton].whileHeld(intakeForwardCommand);
      mButtons[Constants.rightBumperButton].whileHeld(intakeReverseCommand);
    }

    // Shooter
    if (ShooterSubsystem != null) {
      mButtons[Constants.Xbutton].whenPressed(shooterPrime);
      mButtons[Constants.upbutton].whenPressed(new InstantCommand(ShooterSubsystem::cycleAimModeUp, ShooterSubsystem));
      mButtons[Constants.downbutton].whenPressed(new InstantCommand(ShooterSubsystem::cycleAimModeDown, ShooterSubsystem));
    }

    if (CDSSubsystem != null) {
      mButtons[Constants.Xbutton].whileHeld(CDSForwardCommand);
      mButtons[Constants.BButton].whileHeld(CDSReverseCommand);
    }
	// Limelight
	if (driveBaseSubsystem != null && LimelightSubsystem != null) {
      mButtons[Constants.AButton].whenPressed(limelightAlign);
    }
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
    if (driveBaseSubsystem != null) {
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
    return null;
  }


  // TODO: create get methods for other subsystems to pass into TabContainer, or find a more efficient way23
  public static DriveBaseSubsystem getDriveBase() {
    if (driveBaseSubsystem == null) {
      return null;
    }
    return driveBaseSubsystem;
    
  }
}
