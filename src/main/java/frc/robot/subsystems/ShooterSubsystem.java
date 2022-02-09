// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Shooter;

import java.lang.Math;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.common.hardware.MotorController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShooterSubsystem extends SubsystemBase {
  private int aimMode; // 0 is LOW, 1 is AUTO, 2 is LAUNCH, 3 is TARMAC, 4 is TEST
  private MotorController shooter_motorController;
  private MotorController hood_motorController;
  private SparkMaxPIDController KShooterController;
  private SparkMaxPIDController KHoodController;
  private RelativeEncoder KShooterEncoder;
  private RelativeEncoder KHoodEncoder;
  private MotorController cargo_motorController;
  private SparkMaxPIDController kCargoController;
  private RelativeEncoder kCargoEncoder;
  private NetworkTableEntry sbShooterRPM;
  private double currentRPM;

  private double Pconstant;
  private double Iconstant;
  private double Dconstant;
  private double Fconstant;
  private int I_Zone;
  private double MaxOutput;



  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("SmartDashboard");
  private ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");


  private NetworkTableEntry dashTunePid = shooterTab.add("Tune PID", true).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  private NetworkTableEntry PID_P = shooterTab.addPersistent("PID P", Constants.Shooter.kP).withPosition(0, 2).getEntry();
  private NetworkTableEntry PID_I = shooterTab.addPersistent("PID I", Constants.Shooter.kI).withPosition(1, 2).getEntry();
  private NetworkTableEntry PID_D = shooterTab.addPersistent("PID D", Constants.Shooter.kD).withPosition(2, 2).getEntry();
  private NetworkTableEntry PID_F = shooterTab.addPersistent("PID F", Constants.Shooter.kF).withPosition(0,1).getEntry();
  private NetworkTableEntry ShooterReverted = shooterTab.addPersistent("Shooter Reverted", false).withPosition(4,1).getEntry();
  private NetworkTableEntry IShootingMode = shooterTab.add("Shooting Mode",1).withPosition(0, 1).getEntry();
  private NetworkTableEntry DDistance = shooterTab.add("Distance to goal", 0.0).withPosition(1,1).getEntry();
  private NetworkTableEntry DShooterRPM = shooterTab.add("Shooter RPM", 0.0).withPosition(2,1).getEntry();
  private NetworkTableEntry BCargoRunning = shooterTab.add("Is the CDS Running",false).withPosition(3, 1).getEntry();
  private NetworkTableEntry DShooterRPMInput = shooterTab.add("Shooter RPM Input",0.0).withPosition(3, 2).getEntry();
  
  //private NetworkTableEntry PID_F = shooterTab.addPersistent("PID F", Constants.Shooter.).withPosition(3, 2).getEntry();
  //private NetworkTableEntry PID_Izone = shooterTab.addPersistent("PID I Range", Constants.Shooter.).withPosition(4, 2).getEntry();
  //private NetworkTableEntry PID_MaxOutput = shooterTab.addPersistent("PID Peak Output", Constants.Shooter.d).withPosition(5, 2).getEntry();








  public ShooterSubsystem() {

  


    aimMode = 4;
    cargo_motorController = new MotorController("Shooter Cargo", Constants.Shooter.shooterCargoID);
    kCargoController = cargo_motorController.getPID();
    kCargoEncoder = cargo_motorController.getEncoder();

    shooter_motorController = new MotorController("Shooter", Constants.Shooter.shooterID, 40, true);
    KShooterController = shooter_motorController.getPID();
    KShooterEncoder = shooter_motorController.getEncoder();
   

    hood_motorController = new MotorController("Hood", Constants.Shooter.hoodID);
    KHoodController = hood_motorController.getPID();
    KHoodEncoder = shooter_motorController.getEncoder();

  }
  public void updatePID(){

    
    if (dashTunePid.getBoolean(false)){
      
      Pconstant = PID_P.getDouble(0);
      Iconstant = PID_I.getDouble(0);
      Dconstant = PID_D.getDouble(0);
      Fconstant = PID_F.getDouble(0);
      shooter_motorController.getPID().setP(Pconstant);
      shooter_motorController.getPID().setI(Iconstant);
      shooter_motorController.getPID().setD(Dconstant);
      shooter_motorController.getPID().setFF(Fconstant);
    }
  }

  public void adjustHood(double a) {
    KHoodController.setReference(a, CANSparkMax.ControlType.kPosition);

    // Adjusts Hood using PID control to passed angle a
  }

  public double getVelocityInput(){
    return DShooterRPMInput.getDouble(0.0);
  }

  public void windFlywheel(double rpm) {

    // Winds Flywheel using PID control to passed rpm
    // double adjustedRPM = rpm * (Constants.kGearRatioIn / Constants.kGearRatioOut); TODO: reconsider using this
    if(rpm ==0.0){
      KShooterController.setReference(0.0, CANSparkMax.ControlType.kVoltage);
    }
    else
    {
    rpm = DShooterRPMInput.getDouble(0);
    KShooterController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }
    
    if (rpm == shooter_motorController.getEncoder().getVelocity() ){
      runCargo(true);
    }
  }

  public void runCargo(boolean a) {
    if(a){
      cargo_motorController.setSpeed(1.0);
    }else{
      cargo_motorController.setSpeed(0.0);
    }
  }

  public boolean wheelReady(){
    double flywheelSpeed = KShooterEncoder.getVelocity();
    if (flywheelSpeed > currentRPM - 15 && flywheelSpeed < currentRPM + 15) {
      return true;
    }
    return false;
  }

  public void setAimMode(int m) {
    aimMode = m;
  }

  public void cycleAimModeUp() {
    aimMode++;
    if (aimMode > 3) {
      aimMode = 0;
    }
  }

  public void cycleAimModeDown() {
    aimMode--;
    if (aimMode < 0) {
      aimMode = 3;
    }
  }

  public double getTY() {
    // Gets TY, the vertical angle of the target from the limelight
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }
  public double getFlyWheelSpeed(){
    return shooter_motorController.getEncoder().getVelocity();
  }

  public double getDistance() {
    // Uses Limelight to find distance to High Goal
    SmartDashboard.putNumber("ty", getTY());
    return (Constants.Shooter.highHeight - Constants.Shooter.LLHeight) / Math.tan(Math.toRadians((getTY() + Constants.Shooter.LLAngle))); // Return distance in
                                                                                                   // feet
  }


  ///Aimodes
  public void automode(){
    adjustHood(ProjectilePrediction(Constants.Shooter.shooterHeight, 0, Constants.Shooter.highHeight, getDistance(),Constants.Shooter.gravity, Constants.Shooter.airboneTime)[1]);
    windFlywheel((int) (Math.ceil(ProjectilePrediction(Constants.Shooter.shooterHeight, 0, Constants.Shooter.highHeight,getDistance(), 32, Constants.Shooter.airboneTime)[0])));
  }


  public double[] ProjectilePrediction(double y0, double x0, double y, double x, double g, double t) {
    x = x + 1; // Applies an offset to target goal center
    double hoodAngle = Math.toDegrees(Math.atan((y - y0 + 1 / 2 * g * (Math.pow(t, 2))) / x)); //Finds angle to shoot at
    double velocity = Math.abs(x / (Math.cos(Math.toRadians(hoodAngle)) * t)); //Finds velocity in feet per second to shoot at
    double[] returnArray = new double[2];
    returnArray[0] = UnitConversion(velocity, Constants.Shooter.gearDiameter); //Converts FPS to RPM
    returnArray[1] = hoodAngle;
    return returnArray;
  }
  public double UnitConversion(double KBallSpeed, double GearDiameter) {
    // Convert from FPS of the ball into RPM
    return (((KBallSpeed * 12) / Constants.Shooter.gearDiameter) * Constants.Shooter.ballFlywheelratio) * 2;
  }




  public void prime() {
    int aimMode = (int)IShootingMode.getDouble(0);
    // Check what aimMode is active, gets distance if AUTO, winds flywheel, adjusts
    // hood correspondingly
    
    switch (aimMode) {
      case 0: // Case for LOW mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.LOWAngle);
        windFlywheel(Constants.LOWRPM);
        break;
      case 1: // Case for AUTO mode, calculates trajectory and winds flywheel/adjusts hood to
              // a dynamic state
              automode();
       

        break;
      case 2: // Case for LAUNCH mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.LAUNCHAngle);
        windFlywheel(Constants.LAUNCHRPM);
        break;
      case 3: // Case for TARMAC mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.TARMACAngle);
        windFlywheel(Constants.TARMACRPM);
        break;
      case 4: //Case for TEST mode, just takes an RPM and winds
        windFlywheel(3000.0);
        break;
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("dist", getDistance());
    if(DShooterRPM.getDouble(0.0) != shooter_motorController.getEncoder().getVelocity()){
      DShooterRPM.setDouble(shooter_motorController.getEncoder().getVelocity());
    }
    if(DDistance.getDouble(0.0)!= getDistance()){
      DDistance.setDouble(getDistance());
    }
    if (BCargoRunning.getBoolean(false)!= (kCargoEncoder.getVelocity()>1.0)){
      BCargoRunning.setBoolean(kCargoEncoder.getVelocity()>1.0);

    }
    if (dashTunePid.getBoolean(false)){
      updatePID();
      dashTunePid.setBoolean(false);

    }
    if (ShooterReverted.getBoolean(false)!= shooter_motorController.getEncoder().getInverted()){
      shooter_motorController.setInverted(ShooterReverted.getBoolean(false));
    }
  }

}
