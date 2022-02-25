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
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class AutonModes {

  // subsystems
  private DriveBaseSubsystem driveBaseSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private CDSSubsystem cdsSubsystem;
  private IntakeSubsystem intakeSubsystem;

  // Trajectories
  // private Trajectory[] taxiTrajectories;
  private Trajectory[] taxiTrajectory = new Trajectory[1];
  private Trajectory[] oneBallTrajectories;
  private Trajectory[] twoBallTrajectories;
  private Trajectory[] threeBallTrajectories;
  private Trajectory[] fourBallTrajectories;

  // Ramsete Commands, commands for following paths from pathweaver
  private RamseteCommand[] taxiRamseteCommand;
  private RamseteCommand[] oneBallRamseteCommands;
  private RamseteCommand[] twoBallRamseteCommands;
  private RamseteCommand[] threeRamseteCommands;
  private RamseteCommand[] fourRamseteCommands;

  // command groups
  private Command taxiCommand;
  private Command oneBallCommand;
  private Command twoBallCommand;
  private Command twoBallParallel;
  private Command threeBallCommand;
  private Command fourBallCommand;

  private Command resetodometryCommand;

  // this constructor is the default, only needs driveBaseSubsystem, useful when only wanting to
  // test taxi without worrying about other subsystems
  public AutonModes(DriveBaseSubsystem d) {
    this.driveBaseSubsystem = d;

    // parameters are route names (names based on pathweaver, they are located under Auton file)
    // taxiTrajectories = getTrajectories("Taxi");
    initializeTaxiTrajectory();

    // create ramsete commands using the trajectories
    taxiRamseteCommand = getRamseteCommands(taxiTrajectory);

    initializeTaxiMode();
  }

  public AutonModes(
      DriveBaseSubsystem d,
      ShooterSubsystem s,
      LimelightSubsystem l,
      CDSSubsystem c,
      IntakeSubsystem i) {

    this(d);
    this.shooterSubsystem = s;
    this.limelightSubsystem = l;
    this.cdsSubsystem = c;
    this.intakeSubsystem = i;

    // parameters are route names (names based on pathweaver, they are located under Auton file)
    oneBallTrajectories = getTrajectories("OneBall");
    twoBallTrajectories = getTrajectories("TwoBall");
    threeBallTrajectories = getTrajectories("ThreeBall");
    fourBallTrajectories = getTrajectories("FourBall");

    // create ramsete commands using the trajectories
    oneBallRamseteCommands = getRamseteCommands(oneBallTrajectories);
    twoBallRamseteCommands = getRamseteCommands(twoBallTrajectories);
    threeRamseteCommands = getRamseteCommands(threeBallTrajectories);
    fourRamseteCommands = getRamseteCommands(fourBallTrajectories);

    initializeAllOtherGroups(); // already initialized taxi command, now initialize others
  }

  public void initializeTaxiTrajectory() {
    String pathName = "paths/TaxiOut.wpilib.json";
    Path path =
        Filesystem.getDeployDirectory().toPath().resolve(pathName); // goes to scr/main/deploy/paths
    try {
      taxiTrajectory[0] = TrajectoryUtil.fromPathweaverJson(path);
      System.out.println("Success: " + pathName + " created.");
    } catch (IOException e) {
      System.out.println("Trajectory: " + pathName + " not created.");
    }
  }

  public void initializeOtherTrajectories() {}

  private void initializeTaxiMode() {
    taxiCommand =
        new SequentialCommandGroup(
            new WaitCommand(Constants.delaytaxi),
            // taxiRamseteCommands[0]);
            taxiRamseteCommand[0].beforeStarting(
                () -> driveBaseSubsystem.resetOdometry(taxiTrajectory[0].getInitialPose())));
    System.out.println("taxiCommand created");
    System.out.println(taxiCommand.toString());
  }

  private void initializeAllOtherGroups() {
    // TODO: the wait time should not be a constant, should be configurable

    oneBallCommand =
        new SequentialCommandGroup(
            new WaitCommand(Constants.delayshot),
            // new ShooterPrime(shooterSubsystem, limelightSubsystem, cdsSubsystem),
            new ShooterPressed(
                shooterSubsystem,
                limelightSubsystem,
                cdsSubsystem,
                true), // also has a time delay of 2-3 seconds
            new WaitCommand(Constants.delaytaxi),
            oneBallRamseteCommands[0].beforeStarting(
                () -> driveBaseSubsystem.resetOdometry(oneBallTrajectories[0].getInitialPose())));

    twoBallParallel =
        new ParallelDeadlineGroup(
            twoBallRamseteCommands[0].beforeStarting(
                () -> driveBaseSubsystem.resetOdometry(twoBallTrajectories[0].getInitialPose())),
            new IntakeForwardCommand(intakeSubsystem));
    twoBallCommand =
        new SequentialCommandGroup(
            new WaitCommand(Constants.delaytaxi),
            twoBallParallel,
            twoBallRamseteCommands[1], // goes to fender
            new WaitCommand(Constants.delayshot),
            // new ShooterPrime(shooterSubsystem, limelightSubsystem, cdsSubsystem)
            new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, true));

    // threeBallCommand
    // fourBallCommand
  }

  private Trajectory[] getTrajectories(String routeString) {
    // method to intialize multiple trajectories in one auton route
    File f =
        Filesystem.getDeployDirectory()
            .toPath()
            .resolve("Auto/" + routeString)
            .toFile(); // get file with all the names of the path for the auton routine
    System.out.println("File exists: " + f.exists());
    Scanner fileScanner = null;

    try {
      fileScanner = new Scanner(f);
    } catch (FileNotFoundException e) {
      System.out.println("Scanner for file: " + "Autos/" + routeString + " unable to be created");
    }

    List<Trajectory> trajectoryList =
        new ArrayList<
            Trajectory>(); // use list to take in all trajectories, convert it into array later

    if (fileScanner != null) {
      while (fileScanner.hasNext()) {
        String pathName = "paths/" + fileScanner.nextLine().split("\\.")[0] + ".wpilib.json";
        Path path =
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve(pathName); // goes to scr/main/deploy/paths
        Trajectory trajectory;
        try {
          trajectory = TrajectoryUtil.fromPathweaverJson(path);
          trajectoryList.add(trajectory);
          System.out.println("Success: " + pathName + " created.");
        } catch (IOException e) {
          System.out.println("Trajectory: " + pathName + " not created.");
        }
      }
      fileScanner.close();

      /*
      String[] pathNames
      */
    }

    Trajectory[] trajectoryArray = new Trajectory[trajectoryList.size()];
    trajectoryArray = trajectoryList.toArray(trajectoryArray);
    System.out.println("Number of paths: " + trajectoryArray.length);
    return trajectoryArray;
  }

  private RamseteCommand[] getRamseteCommands(Trajectory[] trajectories) {
    RamseteCommand[] ramseteCommands = new RamseteCommand[trajectories.length];

    for (int i = 0; i < ramseteCommands.length; i++) {
      ramseteCommands[i] =
          new RamseteCommand(
              trajectories[i],
              driveBaseSubsystem::getPose,
              new RamseteController(
                  Constants.ramseteB,
                  Constants.ramseteZeta), // ramsete follower to follow trajectory
              Constants.driveKinematics,
              driveBaseSubsystem::acceptWheelSpeeds,
              driveBaseSubsystem);
      System.out.println(ramseteCommands[i].toString());
    }

    System.out.println("RamseteCommand length: " + ramseteCommands.length);
    return ramseteCommands;
  }

  public Command getChosenCommand(String commandName) {
    switch (commandName) {
      case "taxi":
        return taxiCommand;
      case "one Ball":
        return oneBallCommand;
      case "two Ball":
        return twoBallCommand;
      case "three Ball":
        return threeBallCommand;
      case "four Ball":
        return fourBallCommand;
      default:
        return null;
    }
  }
}
