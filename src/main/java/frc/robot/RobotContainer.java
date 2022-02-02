// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.subsystems.DriveBaseSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.ShooterPrime;

 // This class is where the bulk of the robot should be declared. Since Command-based is a
 // "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 // perieodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 // subsystems, commands, and button mappings) should be declared here.

public class RobotContainer {
  public static ShuffleboardTab debugTab;

  // The robot's subsystems and commands are defined here...


  private final Joystick driverJoystick = new Joystick(Constants.portNumber);
  private JoystickButton[] mButtons = new JoystickButton[11];

  // subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem(driverJoystick);
  //private final CDSSubsystem CDSSubsystem = new CDSSubsystem();
  private final IntakeSubsystem IntakeSubsystem = new IntakeSubsystem(); 
  private final ShooterSubsystem ShooterSubsystem = new ShooterSubsystem();

  // commands
  private final DriveBaseTeleopCommand mDriveBaseTeleopCommand = new DriveBaseTeleopCommand(driveBaseSubsystem);
  
  private IntakeForwardCommand intakeForwardCommand = new IntakeForwardCommand(IntakeSubsystem);
  private IntakeReverseCommand intakeReverseCommand = new IntakeReverseCommand(IntakeSubsystem);
  private ShooterPrime shooterPrime = new ShooterPrime(ShooterSubsystem);
  //private CDSForwardCommand CDSForwardCommand = new CDSForwardCommand(CDSSubsystem);
  //private CDSReverseCommand CDSReverseCommand = new CDSReverseCommand(CDSSubsystem);

  // auton
  private Trajectory[] mTrajectories;  // multiple trajectories
  private int trajectoryIndex = 0;
  private Trajectory trajectory;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    debugTab = Shuffleboard.getTab("debug");
    // Configure the button bindings
    for (int i = 1; i < mButtons.length; i++) {
      mButtons[i] = new JoystickButton(driverJoystick, i);
    }


    configureButtonBindings();
    
    try {
      initializeTrajectories();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    driveBaseSubsystem.setDefaultCommand(mDriveBaseTeleopCommand);

  }

  // Use this method to define your button->command mappings. Buttons can be
  // created by
  // instantiating a {@link GenericHID} or one of its subclasses ({@link
  // edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
  // it to a {@link
  // edu.wpi.first.wpilibj2.command.button.JoystickButton}.
  private void configureButtonBindings() {

    // Intake
    mButtons[Constants.leftBumperButton].whileHeld(intakeForwardCommand);
    mButtons[Constants.rightBumperButton].whileHeld(intakeReverseCommand);
    // Shooter
    mButtons[Constants.Xbutton].whenPressed(shooterPrime);
    mButtons[Constants.upbutton].whenPressed(new InstantCommand(ShooterSubsystem::cycleAimModeUp, ShooterSubsystem));
    mButtons[Constants.downbutton].whenPressed(new InstantCommand(ShooterSubsystem::cycleAimModeDown, ShooterSubsystem));
    //mButtons[Constants.Xbutton].whileHeld(CDSForwardCommand);
    //mButtons[Constants.BButton].whileHeld(CDSReverseCommand);
  }

  private void initializeTrajectories() throws IOException {

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


  // TODO: create get methods for other subsystems to pass into TabContainer, or find a more efficient way
  public DriveBaseSubsystem getDriveBase() {
    return driveBaseSubsystem;
    
  }
}
