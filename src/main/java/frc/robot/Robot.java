// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.subsystems.Tabs.TabContainer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


 // The VM is configured to automatically run this class, and to call the functions corresponding to
 // each mode, as described in the TimedRobot documentation. If you change the name of this class or
 // the package after creating this project, you must also update the build.gradle file in the
 // project.

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private TabContainer m_tabContainer;
  private ShuffleboardTab subSystemEnableTab;
  private String[] subsystemList = {"Shooter", "DriveBase", "Intake"};
  private NetworkTableEntry[] sbSubsystemEnables = new NetworkTableEntry[subsystemList.length];
  private boolean[] subsystemEnables = new boolean[subsystemList.length];
  // This function is run when the robot is first started up and should be used for any
  // initialization code.

  // TODO is there a better way to check if a Shuffleboard component already exists?
  private boolean inShuffleboardTab(ShuffleboardTab sbT, String sbTitle) {

    for (ShuffleboardComponent<?> sb : subSystemEnableTab.getComponents()) {
       if (sb.getTitle() == sbTitle)
       {
         return true;
       }
    }
    return false;

  }
   
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    subSystemEnableTab = Shuffleboard.getTab("Subsystem Enables");
    for (int i = 0; i < subSystemEnableTab.getComponents().size(); i++) {
      subSystemEnableTab.getComponents().remove(i);
    }
     NetworkTableEntry sbCommitSubs = subSystemEnableTab.add("Commit Subsystems", false).withSize(2, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    for (int i = 0; i < sbSubsystemEnables.length; i++) {
      sbSubsystemEnables[i] = subSystemEnableTab.add("Enable" + subsystemList[i], false).withSize(1, 1).withPosition(i+2, 0).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    }
    if (sbCommitSubs.setBoolean(false)) {
      System.out.println("Reset Subsystem commit value");

    }
    Shuffleboard.update();
    Shuffleboard.selectTab("Subsystem Enables");
    while(!sbCommitSubs.getBoolean(false)) {
      Shuffleboard.update();
      System.out.println("Waiting for Subsystem enable values");
      try {
        Thread.sleep((long)1000.0);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
    for (int i = 0; i < sbSubsystemEnables.length; i++) {
      subsystemEnables[i] = sbSubsystemEnables[i].getBoolean(false);
    }
    m_robotContainer = new RobotContainer();
    m_tabContainer = new TabContainer(m_robotContainer.getDriveBase());


  }

   // This function is called every robot packet, no matter the mode. Use this for items like
   // diagnostics that you want ran during disabled, autonomous, teleoperated and test.

   // <p>This runs after the mode specific periodic functions, but before LiveWindow and
   // SmartDashboard integrated updating
  
  @Override
  public void robotPeriodic() {
   m_tabContainer.periodic();

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    // schedule driverbaseteleop command
    CommandScheduler.getInstance().run();
  }

  // This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  // This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
   m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  // This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  // This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
