// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.commands.AutonModes;
import frc.robot.commands.CDSForward;
import frc.robot.commands.CDSReverse;
import frc.robot.commands.ClimbPeriodic;
import frc.robot.commands.DeployClimb;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.ShooterHeld;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StopperSubsystem;

// This class is where the bulk of the robot should be declared. Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
// perieodic methods (other than the scheduler calls). Instead, the structure of the robot
// (including
// subsystems, commands, and button mappings) should be declared here.

public class RobotContainer {
  public static ShuffleboardTab debugTab;
  public static ShuffleboardTab controllerDetection;

  // The robot's subsystems and commands are defined here...

  // subsystems
  private static DriveBaseSubsystem driveBaseSubsystem;
  private static IntakeSubsystem intakeSubsystem;
  private static CDSSubsystem CDSSubsystem;
  private static StopperSubsystem stopperSubsystem;
  private static ShooterSubsystem shooterSubsystem;
  private static ClimbSubsystem climbSubsystem;

  // commands
  private DriveBaseTeleopCommand driveBaseTeleopCommand;

  private IntakeForward intakeForwardCommand;
  private CDSForward CDSForwardCommand;
  private ParallelCommandGroup combinedIntake;

  private IntakeReverse intakeReverseCommand;
  private CDSReverse CDSReverseCommand;
  private ParallelCommandGroup combinedOuttake;

  private ShooterHeld shooterHeld;

  private InstantCommand climbEnable;
  private DeployClimb deployClimb;
  private ClimbPeriodic climbPeriodic;

  // auton
  private AutonModes autonModes;
  private Command chosenAutonMode = null;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    debugTab = Shuffleboard.getTab("debug");

    initSubsystems();
    initCommands();

    configureButtonBindings();
  }

  private void initSubsystems() {
    driveBaseSubsystem = new DriveBaseSubsystem(Constants.usingExternal);
    intakeSubsystem = new IntakeSubsystem();
    CDSSubsystem = new CDSSubsystem();
    stopperSubsystem = new StopperSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    climbSubsystem = new ClimbSubsystem();
  }

  private void initCommands() {
    // Initializes commands based on enabled subsystems
      driveBaseTeleopCommand = new DriveBaseTeleopCommand(driveBaseSubsystem, 
        OI.Driver.getDriveSpeedSupplier(), 
        OI.Driver.getDriveRotationSupplier());
      driveBaseSubsystem.setDefaultCommand(driveBaseTeleopCommand);

      intakeForwardCommand = new IntakeForward(intakeSubsystem);
      intakeReverseCommand = new IntakeReverse(intakeSubsystem);

      CDSForwardCommand = new CDSForward(CDSSubsystem, stopperSubsystem);
      CDSReverseCommand = new CDSReverse(CDSSubsystem, stopperSubsystem);

      combinedIntake = new ParallelCommandGroup(intakeForwardCommand, CDSForwardCommand);
      combinedOuttake = new ParallelCommandGroup(intakeReverseCommand, CDSReverseCommand);

      shooterHeld = new ShooterHeld(shooterSubsystem, CDSSubsystem, stopperSubsystem);

    if (climbSubsystem != null) {
      climbEnable = new InstantCommand(climbSubsystem::toggleEnabled, climbSubsystem);
      climbPeriodic = new ClimbPeriodic(climbSubsystem, 
        OI.Operator.getClimbArmSupplier(), 
        OI.Operator.getClimbPoleSupplier());
      deployClimb = new DeployClimb(climbSubsystem);
      climbSubsystem.setDefaultCommand(climbPeriodic);
    }
  }

  // Use this method to define your button->command mappings. Buttons can be
  // created by
  // instantiating a {@link GenericHID} or one of its subclasses ({@link
  // edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
  // it to a {@link
  // edu.wpi.first.wpilibj2.command.button.JoystickButton}.
  private void configureButtonBindings() {
      //Driver Controls
      OI.Driver.getIntakeButton().whileHeld(combinedIntake);
      OI.Driver.getOuttakeButton().whileHeld(combinedOuttake);

      OI.Driver.getShootButton().whileHeld(shooterHeld);

      //Operator Controls
      if (climbSubsystem != null) {
        OI.Operator.getEnableClimbButton().whenPressed(climbEnable);
        OI.Operator.getDeployClimbButton().whileHeld(deployClimb);
      }

      OI.Operator.getCDSForwardButton().whileHeld(CDSForwardCommand);
      OI.Operator.getOuttakeButton().whileHeld(combinedOuttake);
  }

  public Command getAutonomousCommand(Auton mode) {
    switch (mode) {
      case TEST:
        System.out.println(Auton.TEST.getName() + " mode selected.");
        break;
      case PUSHTAXI:
        System.out.println(Auton.PUSHTAXI.getName() + " mode selected.");
        break;
      case INTAKETAXI:
        System.out.println(Auton.INTAKETAXI.getName() + " mode selected.");
        break;
      case ONEBALL:
        System.out.println(Auton.ONEBALL.getName() + " mode selected.");
        break;
      case TWOBALL:
        System.out.println(Auton.TWOBALL.getName() + " mode selected.");
        break;
      case TWOBALLSTEAL1:
        System.out.println(Auton.TWOBALLSTEAL1.getName() + " mode selected.");
        break;
      case TWOBALLSTEAL2:
        System.out.println(Auton.TWOBALLSTEAL2.getName() + " mode selected.");
        break;
      case THREEBALL:
        System.out.println(Auton.THREEBALL.getName() + " mode selected.");
        break;
      case FOURBALL:
        System.out.println(Auton.FOURBALL.getName() + " mode selected.");
        break;
      case FIVEBALL:
        System.out.println(Auton.FIVEBALL.getName() + " mode selected.");
        break;
      default:
        System.out.println("Auton: No mode selected.");
        return null;
    }

    AutonModes autonMode =
        new AutonModes(
            mode,
            driveBaseSubsystem,
            shooterSubsystem,
            CDSSubsystem,
            intakeSubsystem,
            climbSubsystem,
            stopperSubsystem);
    return autonMode.getAutonCommand();
  }

  public void pushSmartDashData() {
    SmartDashboard.putData(climbSubsystem);
    SmartDashboard.putData(driveBaseSubsystem);
    SmartDashboard.putData(CDSSubsystem);
    SmartDashboard.putData(intakeSubsystem);
    SmartDashboard.putData(shooterSubsystem);
  }
}
