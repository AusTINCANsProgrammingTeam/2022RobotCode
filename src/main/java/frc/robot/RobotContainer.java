// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.ShooterModeCycleDown;
import frc.robot.commands.ShooterModeCycleUp;
import frc.robot.commands.ShooterPrime;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


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
  private final DriveBaseSubsystem mDriveBaseSubsystem = new DriveBaseSubsystem(mDriverJoystick);
  private final HopperSubsystem mHopperSubsystem = new HopperSubsystem();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();

  // commands
  private final DriveBaseTeleopCommand mDriveBaseTeleopCommand = new DriveBaseTeleopCommand(mDriveBaseSubsystem);
  private IntakeForwardCommand mIntakeForwardCommand = new IntakeForwardCommand(mIntakeSubsystem);
  private IntakeReverseCommand mIntakeReverseCommand = new IntakeReverseCommand(mIntakeSubsystem);
  private HopperCommand mHopperCommand = new HopperCommand(mHopperSubsystem);
  private ShooterModeCycleDown mShooterModeCycleDown = new ShooterModeCycleDown(mShooterSubsystem);
  private ShooterModeCycleUp mShooterModeCycleUp = new ShooterModeCycleUp(mShooterSubsystem);
  private ShooterPrime mShooterPrimary = new ShooterPrime(mShooterSubsystem);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    debugTab = Shuffleboard.getTab("debug");
    // Configure the button bindings
    for (int i = 1; i < mButtons.length; i++) {
      mButtons[i] = new JoystickButton(mDriverJoystick, i);
    }
    configureButtonBindings();
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
    mButtons[Constants.kAButton].whileHeld(mHopperCommand);

    // Shooter
    mButtons[Constants.kXbutton].whenPressed(mShooterPrimary);
    mButtons[Constants.kUpbutton].whenPressed(mShooterModeCycleUp);
    mButtons[Constants.kDownbutton].whenPressed(mShooterModeCycleDown);
  }

  // Use this to pass the autonomous command to the main {@link Robot} class.
  // @return the command to run in autonomous
  public Command getAutonomousCommand() {
    return null;
    // An ExampleCommand will run in autonomous

  }

  // TODO: create get methods for other subsystems to pass into TabContainer, or find a more efficient way
  public DriveBaseSubsystem getDriveBase() {
    return mDriveBaseSubsystem;
  }
}
