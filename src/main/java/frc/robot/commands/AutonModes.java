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
import java.nio.file.*;
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
  private Trajectory[] taxiTrajectories;
  private Trajectory[] oneBallTrajectories;
  private Trajectory[] twoBallTrajectories;
  private Trajectory[] threeBallTrajectories;
  private Trajectory[] fourBallTrajectories;

  // Ramsete Commands, commands for following paths from pathweaver
  private RamseteCommand[] taxiRamseteCommands;
  private RamseteCommand[] oneBallRamseteCommands;
  private RamseteCommand[] twoBallRamseteCommands;
  private RamseteCommand[] threeRamseteCommands;
  private RamseteCommand[] fourRamseteCommands;

  // command groups
  private Command taxiCommand;
  private Command oneballCommand;
  private Command twoballCommand;
  private Command twoballParallel;
  private Command threeballCommand;
  private Command fourballCommand;

  private Command resetodometryCommand;

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

    // parameters are route names (according to pathweaver, look under Auton file)
    taxiTrajectories = getTrajectories("Taxi");
    oneBallTrajectories = getTrajectories("OneBall");
    twoBallTrajectories = getTrajectories("TwoBall");
    threeBallTrajectories = getTrajectories("ThreeBall");
    fourBallTrajectories = getTrajectories("FourBall");

    taxiRamseteCommands = getRamseteCommands(taxiTrajectories);
    oneBallRamseteCommands = getRamseteCommands(oneBallTrajectories);
    twoBallRamseteCommands = getRamseteCommands(twoBallTrajectories);
    threeRamseteCommands = getRamseteCommands(threeBallTrajectories);
    fourRamseteCommands = getRamseteCommands(fourBallTrajectories);

    // driveBaseSubsystem.resetOdometry(trajectory.getInitialPose());

  }

  private void initializeCommandGroups() {
    // resetodometryCommand = DriveBaseSubsystem.resetOdometry(trajectory.getInitialPose());
    // TODO: the wait time should not be a constant, should be configurable
    taxiCommand =
        new SequentialCommandGroup(new WaitCommand(Constants.delaytaxi), taxiRamseteCommands[0]);

    oneballCommand =
        new SequentialCommandGroup(
            new WaitCommand(Constants.delaytaxi),
            // new ShooterPrime(shooterSubsystem, limelightSubsystem, cdsSubsystem),
            new WaitCommand(Constants.delaytaxi),
            oneBallRamseteCommands[0]);

    twoballParallel =
        new ParallelDeadlineGroup(
            twoBallRamseteCommands[0], new IntakeForwardCommand(intakeSubsystem));
    twoballCommand =
        new SequentialCommandGroup(
            new WaitCommand(Constants.delaytaxi), twoballParallel, twoBallRamseteCommands[1]
            // new ShooterPrime(shooterSubsystem, limelightSubsystem, cdsSubsystem)
            );
  }

  private Trajectory[] getTrajectories(String routeString) {
    // method to intialize multiple trajectories in one auton route
    String routeName = routeString; // TODO: replace route for the correct one on pathweaver
    File f =
        new File(
            "Autos/" + routeName); // get file with all the names of the path for the auton routine
    Scanner fileScanner = null;

    try {
      fileScanner = new Scanner(f);
    } catch (FileNotFoundException e) {
      System.out.println("Scanner for file: " + "Autos/" + routeName + " unable to be created");
    }

    Trajectory[] trajectoryArray;
    List<Trajectory> trajectoryList = new ArrayList<Trajectory>();

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
      } catch (IOException e) {
        System.out.println("Trajectory: " + pathName + " not created.");
      }
    }
    fileScanner.close();

    trajectoryArray = new Trajectory[trajectoryList.size()];
    trajectoryArray = trajectoryList.toArray(trajectoryArray);
    return trajectoryArray;
  }

  private RamseteCommand[] getRamseteCommands(Trajectory[] trajectories) {
    RamseteCommand[] ramseteCommandArray = new RamseteCommand[trajectories.length];

    for (int i = 0; i < ramseteCommandArray.length; i++) {
      ramseteCommandArray[i] =
          new RamseteCommand(
              trajectories[i],
              driveBaseSubsystem::getPose,
              new RamseteController(
                  Constants.ramseteB,
                  Constants.ramseteZeta), // ramsete follower to follow trajectory
              Constants.driveKinematics,
              driveBaseSubsystem::acceptWheelSpeeds,
              driveBaseSubsystem);
    }
    return ramseteCommandArray;
  }

  public Command getChosenCommand(String commandName) {
    switch (commandName) {
      case "taxi":
        return taxiCommand;
      case "one ball":
        return oneballCommand;
      case "two ball":
        return twoballCommand;
      case "three ball":
        return threeballCommand;
      case "four ball":
        return fourballCommand;
      default:
        return null;
    }
  }
}
