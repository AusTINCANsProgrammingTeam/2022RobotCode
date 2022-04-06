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
import frc.robot.Constants.AimModes;
import frc.robot.Constants.Auton;
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

  private Auton mode;
  private Command autonCommand;

  // amount of time in seconds before starting auton, default is 0
  public static double initialWaitTime = Constants.defaultInitialWaitTime;

  // all subsystems come into play
  public AutonModes(
      Auton mode,
      DriveBaseSubsystem drive,
      ShooterSubsystem shooter,
      LimelightSubsystem limelight,
      CDSSubsystem cds,
      IntakeSubsystem intake,
      ClimbSubsystem climb) {

    this.mode = mode;
    this.driveBaseSubsystem = drive;
    this.shooterSubsystem = shooter;
    this.shooterSubsystem.setAimMode(AimModes.ATARMAC);
    this.limelightSubsystem = limelight;
    this.cdsSubsystem = cds;
    this.intakeSubsystem = intake;
    this.climbSubsystem = climb;

    initializeAutonCommand();
  }

  private Trajectory[] getTrajectories(Constants.Auton mode) {
    String[] pathNames = mode.getPaths();

    Trajectory[] trajectories = new Trajectory[pathNames.length];
    for (int i = 0; i < pathNames.length; i++) {
      Path path =
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve(pathNames[i]); // goes to scr/main/deploy/paths
      try {
        trajectories[i] = TrajectoryUtil.fromPathweaverJson(path);
        System.out.println("Success: " + pathNames[i] + " created.");
      } catch (IOException e) {
        System.out.println("Trajectory: " + pathNames[i] + " not created.");
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
      // stopDriveMotors might just be an unnesessary command to the sequence
    }

    return ramseteCommands;
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

  private void initializeAutonCommand() {
    Trajectory[] trajectories = getTrajectories(this.mode);
    Command[] ramsetes = getRamseteCommands(trajectories);
    CommandGroupBase[] parallels = getParallelCommands(ramsetes);

    switch (mode) {
      case TEST:
        int i = 0;
        while (i < ramsetes.length) {
          autonCommand = autonCommand.andThen(ramsetes[i]); // adding ramsetes on to each other
          i++;
        }

      case PUSHTAXI:
        autonCommand =
            new SequentialCommandGroup(
                new WaitCommand(initialWaitTime),
                ramsetes[0]); // this auton mode doesn't need to use intake
        break;

      case INTAKETAXI:
        autonCommand = new SequentialCommandGroup(new WaitCommand(initialWaitTime), parallels[0]);
        break;

      case ONEBALL:
        autonCommand =
            new SequentialCommandGroup(
                new WaitCommand(initialWaitTime),
                new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false),
                parallels[0]);
        break;

      case TWOBALL:
        autonCommand =
            new SequentialCommandGroup(
                new WaitCommand(initialWaitTime),
                parallels[0],
                parallels[1],
                new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false));
        break;

      case TWOBALLSTEAL1:
        autonCommand =
            new SequentialCommandGroup(
                new WaitCommand(initialWaitTime),
                parallels[0],
                parallels[1],
                new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false),
                parallels[2],
                parallels[3],
                new ParallelDeadlineGroup(
                    new WaitCommand(1.5), new OuttakeCommand(intakeSubsystem, cdsSubsystem)));
        break;

      case TWOBALLSTEAL2: // same as twoBallSteal1 for now (placeholder)
        autonCommand =
            new SequentialCommandGroup(
                new WaitCommand(initialWaitTime),
                parallels[0],
                parallels[1],
                new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false),
                parallels[2],
                parallels[3],
                new ParallelDeadlineGroup(
                    new WaitCommand(1.5), new OuttakeCommand(intakeSubsystem, cdsSubsystem)));
        break;

      case THREEBALL:
        autonCommand =
            new SequentialCommandGroup(
                new WaitCommand(initialWaitTime),
                parallels[0],
                parallels[1],
                new ShooterPressed(
                    shooterSubsystem,
                    limelightSubsystem,
                    cdsSubsystem,
                    false), // shoot the two acquired balls
                parallels[2], // grab last ball
                parallels[3], // come back to shoot
                new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false));
        break;

      case FOURBALL:
        autonCommand =
            new SequentialCommandGroup(
                parallels[0],
                parallels[1],
                new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false),
                parallels[2],
                parallels[3],
                new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false));
        break;

      case FIVEBALL: // placeholder as fourball for now
        autonCommand =
            new SequentialCommandGroup(
                parallels[0],
                parallels[1],
                new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false),
                parallels[2],
                parallels[3],
                new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, false));
        break;

      default:
        System.out.println("None of the given auton modes was initialized.");
    }
  }

  public Command getAutonCommand() {
    return autonCommand;
  }

  public static void setWaitTime(double waitTime) {
    initialWaitTime = waitTime; // get value from shuffleboard, units in seconds
  }
}
