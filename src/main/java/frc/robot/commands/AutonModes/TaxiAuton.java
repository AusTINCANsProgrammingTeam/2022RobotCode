package frc.robot.commands.AutonModes;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBaseSubsystem;


public class TaxiAuton {

    private Trajectory trajectory;
    private DriveBaseSubsystem driveBaseSubsystem;
    private RamseteCommand ramseteCommand;
    private Command autonCommand;   // the command group

    public TaxiAuton(DriveBaseSubsystem d) {
        this.driveBaseSubsystem = d;
       
        initializeTrajectory();
        initializeRamsete();
        
        autonCommand = new SequentialCommandGroup(
          new WaitCommand(Constants.delaytaxi),
          ramseteCommand 
        );
    }

    public Command getAutonCommand() {
        return autonCommand;  
    }
    

    private void initializeTrajectory() {
        // auton with just one straight path
        String trajectoryJSON = "paths/Straight.wpilib.json";
        try { 
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON); // goes to scr/main/deploy/paths
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    
        } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }

    // REFERENCE for Ramsete Command

    // if (driveBaseSubsystem != null && trajectory != null) {
    //     // Ramsete Command for Pathweaver
    //     RamseteCommand ramseteCommand =
    //         new RamseteCommand(
    //             trajectory,
    //             driveBaseSubsystem::getPose,
    //             new RamseteController(
    //                 Constants.ramseteB,
    //                 Constants.ramseteZeta), // ramsete follower to follow trajectory
    //             Constants.driveKinematics,
    //             driveBaseSubsystem::acceptWheelSpeeds,
    //             driveBaseSubsystem);
  
    //     driveBaseSubsystem.resetOdometry(trajectory.getInitialPose());
  
    //     return ramseteCommand.andThen(() -> driveBaseSubsystem.acceptWheelSpeeds(0, 0));
    // }

    private void initializeRamsete() {
        ramseteCommand =
            new RamseteCommand(
                trajectory,
                driveBaseSubsystem::getPose,
                new RamseteController(
                    Constants.ramseteB,
                    Constants.ramseteZeta), // ramsete follower to follow trajectory
                Constants.driveKinematics,
                driveBaseSubsystem::acceptWheelSpeeds,
                driveBaseSubsystem);
    }
}
