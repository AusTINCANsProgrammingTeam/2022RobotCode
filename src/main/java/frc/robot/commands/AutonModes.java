package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
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

  private boolean allSubsystemsEnabled;

  // Trajectories
  // private Trajectory[] taxiTrajectories;
  private Trajectory taxiTrajectory;
  private Trajectory oneBallTrajectory;
  private Trajectory twoBallTrajectory;

  private Trajectory[] threeBallTrajectories;
  private Trajectory[] fourBallTrajectories;

  // Ramsete Commands, commands for following paths from pathweaver
  private RamseteCommand taxiRamseteCommand;
  private RamseteCommand oneBallRamseteCommand;
  private RamseteCommand twoBallRamseteCommand;

  private RamseteCommand[] threeRamseteCommands;
  private RamseteCommand[] fourRamseteCommands;

  // command groups
  private Command taxiCommand;
  private Command oneBallCommand;
  private Command twoBallCommand;
  private Command twoBallParallel;
  private Command threeBallCommand;
  private Command fourBallCommand;

  // amount of time before starting auton
  public static double initialWaitTime = 1;

  // this constructor is the default, only needs driveBaseSubsystem, useful when only wanting to
  // test taxi without worrying about other subsystems
  public AutonModes(DriveBaseSubsystem d) {
    this.driveBaseSubsystem = d;

    allSubsystemsEnabled = false;

    // parameters are route names (names based on pathweaver, they are located under Auton file)
    initializeTrajectories();

    // create ramsete commands using the trajectories
    initializeRamseteCommands();

    // initialize the command groups
    initializeCommandGroups();
  }

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

    allSubsystemsEnabled = true;

    // parameters are route names (names based on pathweaver, they are located under Auton file)
    initializeTrajectories();

    // create ramsete commands using the trajectories
    initializeRamseteCommands();

    // initialize the command groups
    initializeCommandGroups();
  }

  private Trajectory getTrajectory(String pathName) {
    Trajectory t = null;
    Path path =
        Filesystem.getDeployDirectory().toPath().resolve(pathName); // goes to scr/main/deploy/paths
    try {
      t = TrajectoryUtil.fromPathweaverJson(path);
      System.out.println("Success: " + pathName + " created.");
    } catch (IOException e) {
      System.out.println("Trajectory: " + pathName + " not created.");
    }
    return t;
  }

  private RamseteCommand getRamseteCommand(Trajectory trajectory) {
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            driveBaseSubsystem::getPose,
            new RamseteController(
                Constants.ramseteB, Constants.ramseteZeta), // ramsete follower to follow trajectory
            Constants.driveKinematics,
            driveBaseSubsystem::acceptWheelSpeeds,
            driveBaseSubsystem);

    return ramseteCommand;
  }

  private void initializeTrajectories() {
    taxiTrajectory = getTrajectory(Constants.taxiPath);

    if (allSubsystemsEnabled) {
      oneBallTrajectory = getTrajectory(Constants.oneBallPath);

      twoBallTrajectory = getTrajectory(Constants.twoBallPath);

      // threeBallTrajectories
      // fourBallTrajectories
    }
  }

  private void initializeRamseteCommands() {
    taxiRamseteCommand = getRamseteCommand(taxiTrajectory);

    if (allSubsystemsEnabled) {
      oneBallRamseteCommand = getRamseteCommand(oneBallTrajectory);

      twoBallRamseteCommand = getRamseteCommand(twoBallTrajectory);

      // threeBallRamseteCommand
      // fourBallRamseteCommand
    }
  }

  private void initializeCommandGroups() {

    taxiCommand =
        new SequentialCommandGroup(
            new WaitCommand(initialWaitTime),
            // taxiRamseteCommands[0]);
            taxiRamseteCommand
                .beforeStarting(
                    () -> driveBaseSubsystem.resetOdometry(taxiTrajectory.getInitialPose()))
                .andThen(() -> driveBaseSubsystem.stopDriveMotors()));

    if (allSubsystemsEnabled) {
      oneBallCommand =
          new SequentialCommandGroup(
              new WaitCommand(initialWaitTime),
              // new ShooterPrime(shooterSubsystem, limelightSubsystem, cdsSubsystem),
              new ShooterPressed(
                  shooterSubsystem,
                  limelightSubsystem,
                  cdsSubsystem,
                  true), // also has a time delay of 2-3 seconds
              new WaitCommand(Constants.delaytaxi),
              oneBallRamseteCommand
                  .beforeStarting(
                      () -> driveBaseSubsystem.resetOdometry(oneBallTrajectory.getInitialPose()))
                  .andThen(() -> driveBaseSubsystem.stopDriveMotors()));

      twoBallParallel =
          new ParallelDeadlineGroup(
              twoBallRamseteCommand.beforeStarting(
                  () ->
                      driveBaseSubsystem.resetOdometry(
                          twoBallTrajectory.getInitialPose())) // go out to get ball
              // new IntakeForwardCommand(intakeSubsystem)
              //     .andThen(() -> driveBaseSubsystem.stopDriveMotors())
              );

      twoBallCommand =
          new SequentialCommandGroup(
              new WaitCommand(initialWaitTime),
              twoBallParallel,
              new WaitCommand(Constants.delayshot),
              // new ShooterPrime(shooterSubsystem, limelightSubsystem, cdsSubsystem)
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, true)
                  .andThen(() -> driveBaseSubsystem.stopDriveMotors()));

      threeBallCommand = null;
      fourBallCommand = null;
    }
  }

  public Command getChosenCommand(String commandName) {
    switch (commandName) {
      case "taxi":
        return taxiCommand;
      case "one ball":
        return oneBallCommand;
      case "two ball":
        return twoBallCommand;
      case "three ball":
        return threeBallCommand;
      case "four ball":
        return fourBallCommand;
      default:
        return null;
    }
  }

  public static void setWaitTime(double waitTime) {
    initialWaitTime = waitTime; // get value from shuffleboard
  }
}
