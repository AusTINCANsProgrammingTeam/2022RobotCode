// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Tabs.TabContainer;

// The VM is configured to automatically run this class, and to call the functions corresponding to
// each mode, as described in the TimedRobot documentation. If you change the name of this class or
// the package after creating this project, you must also update the build.gradle file in the
// project.

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private SendableChooser<Constants.Auton> chooser = new SendableChooser<>();
  private RobotContainer robotContainer;
  private TabContainer tabContainer;
  public UsbCamera usbCamera;

  // This function is run when the robot is first started up and should be used
  // for any
  // initialization code.

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // TODO: Put commands here

    usbCamera = CameraServer.startAutomaticCapture();
    if (isReal()) {
      usbCamera.setResolution(240, 320);
    }

    robotContainer = new RobotContainer();

    chooser.setDefaultOption("Taxi", Constants.Auton.TAXI); // default is taxi mode

    chooser.addOption("Taxi", Constants.Auton.TAXI);
    chooser.addOption("One Ball", Constants.Auton.ONEBALL);
    chooser.addOption("Two Ball", Constants.Auton.TWOBALL);
    chooser.addOption("Three Ball", Constants.Auton.THREEBALL);
    chooser.addOption("Four Ball", Constants.Auton.FOURBALL);
    chooser.addOption("Five Ball", Constants.Auton.FIVEBALL);
    chooser.addOption("Test Mode", Constants.Auton.TEST);

    SmartDashboard.putData(
        "Auto Mode",
        chooser); //  TODO: find a way to put it into desired specific named tabs such as "Auton"

    if (RobotContainer.getDriveBase() != null) {
      tabContainer = new TabContainer(RobotContainer.getDriveBase());
    }
  }

  // This function is called every robot packet, no matter the mode. Use this for items like
  // diagnostics that you want ran during disabled, autonomous, teleoperated and test.

  // <p>This runs after the mode specific periodic functions, but before LiveWindow and
  // SmartDashboard integrated updating

  @Override
  public void robotPeriodic() {
    if (tabContainer != null) {
      tabContainer.periodic();
    }

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  // This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // This would put the command that the auto mode is using on the smart dashboard, for debugging
    // SmartDashboard.putString("Auto Command", chooser.getSelected().getName());
  }

  // This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.initAuton(chooser.getSelected());
    autonomousCommand = robotContainer.getAutonomousCommand(chooser.getSelected());

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
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
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
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
