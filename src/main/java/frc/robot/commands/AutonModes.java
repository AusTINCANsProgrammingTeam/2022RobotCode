package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.io.IOException;
import java.nio.file.Path;

public class AutonModes {

  // subsystems
  private DriveBaseSubsystem driveBaseSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private CDSSubsystem cdsSubsystem;
  private IntakeSubsystem intakeSubsystem;

  private boolean shooterEnabled;

  // Ramsete Commands, commands for following paths from pathweaver
  private Command taxiRamseteCommand;
  private Command oneBallRamseteCommand;
  private Command[] twoBallRamseteCommands;
  private Command[] threeBallRamseteCommands;
  private Command[] fourBallRamseteCommands;
  private Command[] fiveBallRamseteCommands;

  private Command[] testRamseteCommands; // for testing

  // command groups
  private Command taxiCommand;
  private Command oneBallCommand;
  // --------------------------
  private Command twoBallCommand;
  private Command twoBallParallel;
  // -----------------------------
  private Command threeBallParallel;
  private Command threeBallCommand;
  // -------------------------------
  private Command fourBallCommand;
  private Command fiveBallCommand;

  private Command testCommand; // for testing miscellaneous: for example single ramsete commands

  // amount of time in seconds before starting auton
  public static double initialWaitTime = 1;

  // constructor used when only need to use driveBase, for example when testing testCommand
  public AutonModes(DriveBaseSubsystem d) {

    this.driveBaseSubsystem = d;

    initializeTest(); // initializes both ramsete and the command group
  }

  // constructor that doesn't use shooter
  public AutonModes(DriveBaseSubsystem d, IntakeSubsystem i, CDSSubsystem c) {
    this.driveBaseSubsystem = d;
    this.intakeSubsystem = i;
    this.cdsSubsystem = c;

    shooterEnabled = false;

    // create ramsete commands using the trajectories
    initializeRamseteCommands();

    // initialize the command groups
    initializeCommandGroups();
  }

  // all subsystems come into play
  public AutonModes(
      DriveBaseSubsystem d,
      ShooterSubsystem s,
      LimelightSubsystem l,
      CDSSubsystem c,
      IntakeSubsystem i) {

    this.driveBaseSubsystem = d;
    this.shooterSubsystem = s;
    this.limelightSubsystem = l;
    this.cdsSubsystem = c;
    this.intakeSubsystem = i;

    shooterEnabled = true;

    // create ramsete commands using the trajectories
    initializeRamseteCommands();

    // initialize the command groups
    initializeCommandGroups();
  }

  private Trajectory[] getTrajectories(String... pathName) {
    Trajectory[] trajectories = new Trajectory[pathName.length];
    for (int i = 0; i < pathName.length; i++) {
      Path path =
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve(pathName[i]); // goes to scr/main/deploy/paths
      try {
        trajectories[i] = TrajectoryUtil.fromPathweaverJson(path);
        System.out.println("Success: " + pathName[i] + " created.");
      } catch (IOException e) {
        System.out.println("Trajectory: " + pathName[i] + " not created.");
      }
    }
    return trajectories;
  }

  private Command[] getRamseteCommands(Trajectory... trajectories) {
    Command[] ramseteCommands = new Command[trajectories.length];
    for (int i = 0; i < trajectories.length; i++) {
      RamseteCommand r =
          new RamseteCommand(
              trajectories[i],
              driveBaseSubsystem::getPose,
              new RamseteController(
                  Constants.ramseteB,
                  Constants.ramseteZeta), // ramsete follower to follow trajectory
              Constants.driveKinematics,
              driveBaseSubsystem::acceptWheelSpeeds,
              driveBaseSubsystem);

      // first ramsete command needs to have driveBase reset odometry to match that of pathweaver
      if (i == 0) {
        Pose2d p = trajectories[i].getInitialPose();
        ramseteCommands[i] = r.beforeStarting(() -> driveBaseSubsystem.resetOdometry(p));
      } else {
        ramseteCommands[i] = r;
      }
    }

    return ramseteCommands;
  }

  private void initializeRamseteCommands() {
    taxiRamseteCommand =
        getRamseteCommands(getTrajectories(Constants.Auton.TAXI.getPaths()))[
            0]; // only has one trajectory so only one ramsete command (first element of array)

    if (shooterEnabled) {
      oneBallRamseteCommand =
          getRamseteCommands(getTrajectories(Constants.Auton.ONEBALL.getPaths()))[0];

      twoBallRamseteCommands =
          getRamseteCommands(getTrajectories(Constants.Auton.TWOBALL.getPaths()));

      threeBallRamseteCommands =
          getRamseteCommands(getTrajectories(Constants.Auton.THREEBALL.getPaths()));

      // fourBallRamseteCommand
      // fiveBallRamseteCommand
    }
  }

  private void initializeCommandGroups() {
    taxiCommand =
        new SequentialCommandGroup(
            new DeployIntake(intakeSubsystem, cdsSubsystem), // deploy/extend the intake
            new WaitCommand(initialWaitTime), // wait before starting, units in seconds
            taxiRamseteCommand.andThen(() -> driveBaseSubsystem.stopDriveMotors()));

    if (shooterEnabled) {
      oneBallCommand =
          new SequentialCommandGroup(
              new DeployIntake(intakeSubsystem, cdsSubsystem),
              new WaitCommand(initialWaitTime),
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false),
              new WaitCommand(Constants.delaytaxi),
              oneBallRamseteCommand.andThen(() -> driveBaseSubsystem.stopDriveMotors()));

      twoBallParallel =
          new ParallelDeadlineGroup(
              twoBallRamseteCommands[0].andThen(
                  () -> driveBaseSubsystem.stopDriveMotors()), // travel to get ball
              new IntakeForwardCommand(intakeSubsystem, cdsSubsystem));

      twoBallCommand =
          new SequentialCommandGroup(
              new DeployIntake(intakeSubsystem, cdsSubsystem),
              new WaitCommand(initialWaitTime),
              twoBallParallel,
              twoBallRamseteCommands[1].andThen(() -> driveBaseSubsystem.stopDriveMotors()),
              new WaitCommand(Constants.delayshot),
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false));

      threeBallParallel =
          new ParallelDeadlineGroup(
              threeBallRamseteCommands[0].andThen(
                  () -> driveBaseSubsystem.stopDriveMotors()), // travel to get the two balls
              new IntakeForwardCommand(intakeSubsystem, cdsSubsystem));

      threeBallCommand =
          new SequentialCommandGroup(
              new DeployIntake(intakeSubsystem, cdsSubsystem),
              new WaitCommand(initialWaitTime),
              new ShooterPressed(
                  shooterSubsystem, limelightSubsystem, cdsSubsystem, false), // shoot preloaded
              threeBallParallel,
              threeBallRamseteCommands[1].andThen(() -> driveBaseSubsystem.stopDriveMotors()),
              new ShooterPressed(
                  shooterSubsystem,
                  limelightSubsystem,
                  cdsSubsystem,
                  false)); // shoot the two acquired balls

      // fourBallCommand
      // fiveBallCommand

    }
  }

  private void initializeTest() {
    // REPLACE ME to test anything
    testRamseteCommands = getRamseteCommands(getTrajectories(Constants.Auton.TEST.getPaths()));
    testCommand =
        new SequentialCommandGroup(
            testRamseteCommands[0].andThen(() -> driveBaseSubsystem.stopDriveMotors()),
            testRamseteCommands[1].andThen(() -> driveBaseSubsystem.stopDriveMotors()));
  }

  public Command getChosenCommand(Constants.Auton mode) {
    switch (mode) {
      case TEST:
        System.out.println("Auton: Test mode selected.");
        return testCommand;
      case TAXI:
        System.out.println("Auton: Taxi mode selected.");
        return taxiCommand;
      case ONEBALL:
        System.out.println("Auton: One Ball mode selected.");
        return oneBallCommand;
      case TWOBALL:
        System.out.println("Auton: Two Ball mode selected.");
        return twoBallCommand;
      case THREEBALL:
        System.out.println("Auton: Three Ball mode selected.");
        return threeBallCommand;
      case FOURBALL:
        System.out.println("Auton: Four Ball mode selected.");
        return fourBallCommand;
      case FIVEBALL:
        System.out.println("Auton: Five Ball mode selected.");
        return fiveBallCommand;
      default:
        System.out.println("Auton: No mode selected.");
        return null;
    }
  }

  public static void setWaitTime(double waitTime) {
    initialWaitTime = waitTime; // get value from shuffleboard, units in seconds
  }
}
