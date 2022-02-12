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
  private static final Joystick driverJoystick = new Joystick(Constants.portNumber0);
  private JoystickButton[] buttons = new JoystickButton[11];


  // subsystems
  private static DriveBaseSubsystem driveBaseSubsystem;
  private static CDSSubsystem CDSSubsystem;
  private static IntakeSubsystem intakeSubsystem; 
  private static ShooterSubsystem shooterSubsystem;
  private static LimelightSubsystem limelightSubsystem;

  // commands
  private static DriveBaseTeleopCommand driveBaseTeleopCommand;
  private IntakeForwardCommand intakeForwardCommand;
  private IntakeReverseCommand intakeReverseCommand;
  private ShooterPrime shooterPrime;
  private CDSForwardCommand CDSForwardCommand;
  private CDSReverseCommand CDSReverseCommand; 
  private LimelightAlign limelightAlign;
  private BeamBreakCommand beamBreakCommand;

  
  private Trajectory[] mTrajectories;  // multiple trajectories
  private int trajectoryIndex = 0;
  private Trajectory trajectory;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    debugTab = Shuffleboard.getTab("debug");

    // Configure the button bindings
    for (int i = 1; i < buttons.length; i++) {
      buttons[i] = new JoystickButton(driverJoystick, i);
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
<<<<<<< HEAD
    //Initializes commands based on enabled subsystems
      if(driveBaseSubsystem != null){
        driveBaseTeleopCommand = new DriveBaseTeleopCommand(driveBaseSubsystem);
        driveBaseSubsystem.setDefaultCommand(driveBaseTeleopCommand);
      }
      if(CDSSubsystem != null && shooterSubsystem != null){
        CDSForwardCommand = new CDSForwardCommand(CDSSubsystem);
        CDSReverseCommand = new CDSReverseCommand(CDSSubsystem);
        buttons[Constants.rightBumperButton].whileHeld(CDSForwardCommand);
        buttons[Constants.leftBumperButton].whileHeld(CDSReverseCommand);
      }
      if(intakeSubsystem != null){
        intakeForwardCommand = new IntakeForwardCommand(intakeSubsystem);
        intakeReverseCommand = new IntakeReverseCommand(intakeSubsystem);
        beamBreakCommand = new BeamBreakCommand(intakeSubsystem);
        buttons[Constants.leftBumperButton].whileHeld(intakeForwardCommand);
        buttons[Constants.rightBumperButton].whileHeld(intakeReverseCommand);
      }
      if(shooterSubsystem != null && limelightSubsystem != null && CDSSubsystem != null){
        shooterPrime = new ShooterPrime(shooterSubsystem, limelightSubsystem, CDSSubsystem);
        buttons[Constants.rightTriggerButton].whenPressed(shooterPrime);
        buttons[Constants.upPOV].whenPressed(new InstantCommand(shooterSubsystem::cycleAimModeUp, shooterSubsystem));
        buttons[Constants.downPOV].whenPressed(new InstantCommand(shooterSubsystem::cycleAimModeDown, shooterSubsystem));
      }
      if(limelightSubsystem != null && driveBaseSubsystem != null){
        limelightAlign = new LimelightAlign(limelightSubsystem, driveBaseSubsystem);
      }
=======

    // Intake
    if(intakeSubsystem != null) {
      buttons[Constants.LBumper].whileHeld(intakeForwardCommand);
      buttons[Constants.RBumper].whileHeld(intakeReverseCommand);
    }

    // Shooter
    if (shooterSubsystem != null) {
      buttons[Constants.backButton].whenPressed(shooterPrime);
      buttons[Constants.LJoystickButton].whenPressed(new InstantCommand(shooterSubsystem::cycleAimModeUp, shooterSubsystem));
      buttons[Constants.RJoystickButton].whenPressed(new InstantCommand(shooterSubsystem::cycleAimModeDown, shooterSubsystem));
    }

    //CDS
    if (CDSSubsystem != null) {
      buttons[Constants.LTriggerButton].whileHeld(CDSForwardCommand);
      buttons[Constants.RTriggerButton].whileHeld(CDSReverseCommand);
    }
	// Limelight
	if (driveBaseSubsystem != null && limelightSubsystem != null) {
      buttons[Constants.startButton].whenPressed(limelightAlign);
    }
>>>>>>> origin/main
  }

  private void initializeTrajectories() throws IOException {
    String[] trajectoryJSON = {"One.wpilib.json", "Two.wpilib.json", "Three.wpilib.json", "Four.wpilib.json"};  // add new trajectories manually
    mTrajectories = new Trajectory[trajectoryJSON.length];
    for(int i = 0; i < trajectoryJSON.length; i++) {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON[i]);
    Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    mTrajectories[i] = trajectory;
    }

    // to test auton with just a one straight path
    //String trajectoryJSON = "Straight.wpilib.json";
    //Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
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
