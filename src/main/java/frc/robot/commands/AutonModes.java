package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
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
  private ClimbSubsystem climbSubsystem;

  private boolean shooterEnabled;

  // Ramsete Commands, commands for following paths from pathweaver
  private Command pushTaxiRamseteCommand;
  private Command intakeTaxiRamseteCommand;
  private Command oneBallRamseteCommand;
  private Command[] twoBallRamseteCommands;
  private Command[] threeBallRamseteCommands;
  private Command[] fourBallRamseteCommands;
  private Command[] fiveBallRamseteCommands;

  private Command[] driveTestRamseteCommands; // for testing

  // command groups
  private Command pushTaxiCommand;
  private Command intakeTaxiCommand;
  private Command oneBallCommand;
  private Command twoBallCommand;
  private Command threeBallCommand;
  private Command fourBallCommand;
  private Command fiveBallCommand;

  private Command
      driveTestCommand; // for testing miscellaneous: for example single ramsete commands

  // amount of time in seconds before starting auton, default is 0
  public static double initialWaitTime = Constants.defaultInitialWaitTime;

  // constructor used when only need to use driveBase, for example when testing
  // driveTestCommand
  public AutonModes(DriveBaseSubsystem d) {

    this.driveBaseSubsystem = d;

    initializeTest(); // initializes both ramsete and the command group
  }

  // constructor that doesn't use shooter (for taxi auton)
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
      IntakeSubsystem i,
      ClimbSubsystem t) {

    this.driveBaseSubsystem = d;
    this.shooterSubsystem = s;
    this.shooterSubsystem.setAimMode(Constants.AimModes.ATARMAC);
    this.limelightSubsystem = l;
    this.cdsSubsystem = c;
    this.intakeSubsystem = i;
    this.climbSubsystem = t;

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

      // first ramsete command needs to have driveBase reset odometry to match that of
      // pathweaver
      if (i == 0) {
        Pose2d p = trajectories[i].getInitialPose();
        ramseteCommands[i] = r.beforeStarting(() -> driveBaseSubsystem.resetOdometry(p));
      } else {
        ramseteCommands[i] = r;
      }
      ramseteCommands[i] =
          ramseteCommands[i].andThen(
              () -> driveBaseSubsystem.stopDriveMotors()); // stop drive motors after each ramsete
    }

    return ramseteCommands;
  }

  private void initializeRamseteCommands() {
    // only has one trajectory so only one ramsete command (first element of array)
    pushTaxiRamseteCommand =
        getRamseteCommands(getTrajectories(Constants.Auton.PUSHTAXI.getPaths()))[0];
    intakeTaxiRamseteCommand =
        getRamseteCommands(getTrajectories(Constants.Auton.INTAKETAXI.getPaths()))[0];

    if (shooterEnabled) {
      oneBallRamseteCommand =
          getRamseteCommands(getTrajectories(Constants.Auton.ONEBALL.getPaths()))[0];

      twoBallRamseteCommands =
          getRamseteCommands(getTrajectories(Constants.Auton.TWOBALL.getPaths()));

      threeBallRamseteCommands =
          getRamseteCommands(getTrajectories(Constants.Auton.THREEBALL.getPaths()));

      fourBallRamseteCommands =
          getRamseteCommands(getTrajectories(Constants.Auton.FOURBALL.getPaths()));
      fiveBallRamseteCommands =
          getRamseteCommands(getTrajectories(Constants.Auton.FIVEBALL.getPaths()));
    }
  }

  private CommandGroupBase[] getParallelCommands(Command... ramsetes) {
    CommandGroupBase[] parallels = new CommandGroupBase[ramsetes.length];
    for (int i = 0; i < ramsetes.length; i++) {
      parallels[i] =
          new ParallelDeadlineGroup(
              ramsetes[i],
              new CombinedIntakeCDSForwardCommand(intakeSubsystem, cdsSubsystem, shooterSubsystem));
    }
    parallels[0] =
        parallels[0].alongWith(new InstantCommand(climbSubsystem::deployHA, climbSubsystem));

    return parallels;
  }

  private void initializeCommandGroups() {

    pushTaxiCommand =
        new SequentialCommandGroup(new WaitCommand(initialWaitTime), pushTaxiRamseteCommand);

    // ---------------------------------------------

    CommandGroupBase intakeTaxiParallel = getParallelCommands(intakeTaxiRamseteCommand)[0];
    intakeTaxiCommand =
        new SequentialCommandGroup(new WaitCommand(initialWaitTime), intakeTaxiParallel);

    // --------------------------------------

    if (shooterEnabled) {

      oneBallCommand =
          new SequentialCommandGroup(
              new WaitCommand(initialWaitTime),
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false),
              oneBallRamseteCommand);

      // -------------------------------------------

      CommandGroupBase[] twoBallParallels = getParallelCommands(twoBallRamseteCommands);
      twoBallCommand =
          new SequentialCommandGroup(
              new WaitCommand(initialWaitTime), twoBallParallels[0], twoBallParallels[1]);

      // -------------------------------------------

      CommandGroupBase[] threeBallParallels = getParallelCommands(threeBallRamseteCommands);
      threeBallCommand =
          new SequentialCommandGroup(
              new WaitCommand(initialWaitTime),
              threeBallParallels[0],
              threeBallParallels[1],
              new ShooterPressed(
                  shooterSubsystem,
                  limelightSubsystem,
                  cdsSubsystem,
                  false), // shoot the two acquired balls
              threeBallParallels[2], // grab last ball
              threeBallParallels[3], // come back to shoot
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false));

      // --------------------------------------------

      CommandGroupBase[] fourBallParallels = getParallelCommands(fourBallRamseteCommands);
      // similar path to threeball, now just getting the additional ball at terminal
      fourBallCommand =
          new SequentialCommandGroup(
              fourBallParallels[0],
              fourBallParallels[1],
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false),
              fourBallParallels[2],
              fourBallParallels[3],
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false));

      // ---------------------------------------

      CommandGroupBase[] fiveBallParallels = getParallelCommands(fiveBallRamseteCommands);
      fiveBallCommand =
          new SequentialCommandGroup(
              new WaitCommand(initialWaitTime),
              fiveBallParallels[0],
              fiveBallParallels[1],
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false),
              fiveBallParallels[2],
              fiveBallParallels[3],
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false),
              fiveBallParallels[4],
              fiveBallParallels[5],
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false));
    }
  }

  private void initializeTest() {
    // REPLACE ME to test anything
    driveTestRamseteCommands = getRamseteCommands(getTrajectories(Constants.Auton.TEST.getPaths()));
    driveTestCommand = new WaitCommand(initialWaitTime);
    int i = 0;
    while (i < driveTestRamseteCommands.length) {
      driveTestCommand =
          driveTestCommand.andThen(driveTestRamseteCommands[i]); // adding next ramsete
      i++;
    }
  }

  public Command getChosenCommand(Constants.Auton mode) {
    switch (mode) {
      case TEST:
        System.out.println("Auton: Test mode selected.");
        return driveTestCommand;
      case PUSHTAXI:
        System.out.println("Auton: Push Taxi mode selected.");
        return pushTaxiCommand;
      case INTAKETAXI:
        System.out.println("Auton: Intake Taxi mode selected.");
        return intakeTaxiCommand;
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
