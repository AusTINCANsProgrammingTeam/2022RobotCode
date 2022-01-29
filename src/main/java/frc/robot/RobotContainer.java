// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.subsystems.DriveBaseSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Map;
import java.util.concurrent.Flow.Subscriber;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
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


  private final Joystick mDriverJoystick = new Joystick(Constants.kPortNumber);
  private JoystickButton[] mButtons = new JoystickButton[11];

  // subsystems
  private final DriveBaseSubsystem mDriveBaseSubsystem;
  private final CDSSubsystem mCDSSubsystem;
  private final IntakeSubsystem mIntakeSubsystem; 
  private final ShooterSubsystem mShooterSubsystem;

  // commands
  private final DriveBaseTeleopCommand mDriveBaseTeleopCommand;
  
  private IntakeForwardCommand mIntakeForwardCommand;
  private IntakeReverseCommand mIntakeReverseCommand;
  private ShooterPrime mShooterPrime;
  private CDSForwardCommand mCDSForwardCommand;
  private CDSReverseCommand mCDSReverseCommand;

  // auton
  // private Trajectory[] mTrajectories;  // multiple trajectories
  // private int trajectoryIndex = 0;
  private Trajectory trajectory;

  private String[] subsystemList = {"CDS", "Shooter", "DriveBase", "Intake"};
  private Map<String, NetworkTableEntry> sbSubsystemEnables = new Hashtable<String, NetworkTableEntry>();
  private Map<String, Boolean> subsystemEnables = new Hashtable<String, Boolean>();

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    debugTab = Shuffleboard.getTab("debug");
    for (String s : subsystemList) {
      sbSubsystemEnables.put(s,debugTab.add("Enable" + s, false).withSize(1, 1).withPosition(i, 0).withWidget(BuiltInWidgets.kToggleSwitch).getEntry());
      subsystemEnables.put(s, Boolean.valueOf(false));
    }
    Shuffleboard.selectTab("debug");
    Shuffleboard.update();
    for (String s : subsystemList) {
      subsystemEnables.put(s, Boolean.valueOf(sbSubsystemEnables.get(s).getBoolean(false)));
    }

    // Configure the button bindings
    for (int i = 1; i < mButtons.length; i++) {
      mButtons[i] = new JoystickButton(mDriverJoystick, i);
    }

    // subsystems
    if (subsystemEnables.get("Drivebase")) {
      mDriveBaseSubsystem = new DriveBaseSubsystem(mDriverJoystick);
      mDriveBaseTeleopCommand = new DriveBaseTeleopCommand(mDriveBaseSubsystem);
      mDriveBaseSubsystem.setDefaultCommand(mDriveBaseTeleopCommand);
    }
    if (subsystemEnables.get("CDS")) {
      mCDSSubsystem = new CDSSubsystem();
      mCDSForwardCommand = new CDSForwardCommand(mCDSSubsystem);
      mCDSReverseCommand = new CDSReverseCommand(mCDSSubsystem);
    }
    if (subsystemEnables.get("Intake")) {
      mIntakeSubsystem = new IntakeSubsystem(); 
      mIntakeForwardCommand = new IntakeForwardCommand(mIntakeSubsystem);
      mIntakeReverseCommand = new IntakeReverseCommand(mIntakeSubsystem);
    }
    if (subsystemEnables.get("Shooter")) {
      mShooterSubsystem = new ShooterSubsystem();
      mShooterPrime = new ShooterPrime(mShooterSubsystem);
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
    mButtons[Constants.kLeftBumperButton].whileHeld(mIntakeForwardCommand);
    mButtons[Constants.kRightBumperButton].whileHeld(mIntakeReverseCommand);

    // Shooter
    if (mShooterSubsystem != null) {
      mButtons[Constants.kXbutton].whenPressed(mShooterPrime);
      mButtons[Constants.kUpbutton].whenPressed(new InstantCommand(mShooterSubsystem::cycleAimModeUp, mShooterSubsystem));
      mButtons[Constants.kDownbutton].whenPressed(new InstantCommand(mShooterSubsystem::cycleAimModeDown, mShooterSubsystem));
    }

    if (mCDSSubsystem != null) {
      mButtons[Constants.kXButton].whileHeld(mCDSForwardCommand);
      mButtons[Constants.kBButton].whileHeld(mCDSReverseCommand);
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
    if (mDriveBaseSubsystem != null) {
      //Ramsete Command for Pathweaver
      RamseteCommand ramseteCommand =
      new RamseteCommand(
          trajectory,
          mDriveBaseSubsystem::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), //Fix these constants by
                                                                              //characterizing the robot
          new SimpleMotorFeedforward(
              Constants.ksVolts,
              Constants.kvVoltSecondsPerMeter,
              Constants.kaVoltSecondsSquaredPerMeter),

          Constants.kDriveKinematics,
          
          mDriveBaseSubsystem::getWheelSpeeds,
          new PIDController(1, 0, 0),
          new PIDController(1, 0, 0),
          //RamseteCommand passes volts to the callback
          mDriveBaseSubsystem::setAutonVolts,
          mDriveBaseSubsystem);
          
      mDriveBaseSubsystem.resetOdometry(trajectory.getInitialPose());

      return ramseteCommand.andThen(() -> mDriveBaseSubsystem.setAutonVolts(0,0));
    }
    return null;
  }


  // TODO: create get methods for other subsystems to pass into TabContainer, or find a more efficient way
  public DriveBaseSubsystem getDriveBase() {
    return mDriveBaseSubsystem;
    
  }
}
