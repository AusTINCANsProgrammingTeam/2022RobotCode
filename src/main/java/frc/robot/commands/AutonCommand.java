package frc.robot.commands;

import java.util.List;

import frc.robot.subsystems.DriveBaseSubsystem;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.RamseteController;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

public class AutonCommand {

    private DriveBaseSubsystem mDriveBaseSubsystem;
    private List<Trajectory> mTrajectories;  // multiple trajectories
    private int trajectoryIndex = 0;
    private Trajectory trajectory;

    public AutonCommand(DriveBaseSubsystem d) {
        this.mDriveBaseSubsystem = d;
        initializeTrajectories();
    }

    private void initializeTrajectories() {
        // auton with just a one straight path
        String trajectoryJSON = "paths/RightCurve.wpilib.json";
        try { 
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON); // goes to scr/main/deploy/paths
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    
        } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    
        // method to intialize multiple trajectories in one auton route
        // TODO: some sort of sendable chooser on shuffle board get the desire route
        String routeName = "RouteA";
    
        File f = new File("Autos/" + routeName);  // get file with all the names of the path for the auton routine
        Scanner fileScanner;
        List<String> trajectoryJSONList = new ArrayList<String>();
    
        try {
          fileScanner = new Scanner(f);
          while(fileScanner.hasNextLine()) {
            trajectoryJSONList.add(fileScanner.nextLine().split("\\.")[0]);  // get name of file not the (.path)
          }
    
          if(trajectoryJSONList != null) {
            for(String name : trajectoryJSONList) {
              String pathName = "paths/" + name + ".wpilib.json";
              Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathName); // goes to scr/main/deploy/paths
              Trajectory trajectory;
              try {
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                mTrajectories.add(trajectory);
              } catch (IOException e) {
                System.out.println("***Unable to create individual trajectories.");
              }
            }
          }  
    
          fileScanner.close();
        } catch (FileNotFoundException e) {
          System.out.println("***Trajectory List unable to be created.");
        }
        
    }
}
