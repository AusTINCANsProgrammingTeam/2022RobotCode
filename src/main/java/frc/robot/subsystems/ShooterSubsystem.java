// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;
import frc.robot.Constants.Shooter;


import frc.robot.RobotContainer;
import frc.robot.Constants.Shooter;

import java.lang.Math;

import com.fasterxml.jackson.annotation.JacksonInject.Value;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.common.hardware.MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShooterSubsystem extends SubsystemBase {
  private double aimMode; // 0 is LOW, 1 is AUTO, 2 is LAUNCH, 3 is TARMAC, 4 is TEST
  private MotorController shooter_motorController;
  private MotorController hood_motorController;
  private SparkMaxPIDController KShooterController;
  private SparkMaxPIDController KHoodController;
  private RelativeEncoder KShooterEncoder;
  private RelativeEncoder KHoodEncoder;
  private MotorController cargo_motorController;
  private RelativeEncoder kCargoEncoder;
  private SparkMaxPIDController kCargoController;
  private double currentRPM;

  private double Pconstant;
  private double Iconstant;
  private double Dconstant;
  private double Fconstant;
  private double IMaxAccumconstant;
  private int IMaxAccumIDconstant;
  private int I_Zone;
  private double MaxOutput;






  private ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter Tab");


  private NetworkTableEntry dashTunePid = shooterTab.add("Tune PID", true).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  private NetworkTableEntry PID_P = shooterTab.add("PID P", Constants.Shooter.kP).withPosition(0, 1).getEntry();
  private NetworkTableEntry PID_I = shooterTab.add("PID I", Constants.Shooter.kI).withPosition(0, 2).getEntry();
  private NetworkTableEntry PID_D = shooterTab.add("PID D", Constants.Shooter.kD).withPosition(0, 3).getEntry();
  private NetworkTableEntry PID_F = shooterTab.add("PID F", Constants.Shooter.kF).withPosition(0,4).getEntry();
  private NetworkTableEntry PID_IMaxAccum = shooterTab.add("PID I Max Accum", Constants.Shooter.kMaxI).withPosition(0,5).getEntry();
  private NetworkTableEntry PID_Izone = shooterTab.add("PID I Range", Constants.Shooter.kIZone).withPosition(1, 0).getEntry();
  private NetworkTableEntry PID_MaxOutput = shooterTab.add("PID Peak Output", Constants.Shooter.kMaxOutput).withPosition(1, 1).getEntry();
  private NetworkTableEntry PID_MinOutput = shooterTab.add("PID Min Out ", Constants.Shooter.kMinOutput).withPosition(1,2).getEntry();
  private NetworkTableEntry PID_IMaxAccumID = shooterTab.add("PID Peak Output Slot ID", Constants.Shooter.kMaxISlotId).withPosition(1,3).getEntry();

  private NetworkTableEntry ShooterReverted = shooterTab.add("Shooter Reverted", false).withPosition(1,4).getEntry();
  private NetworkTableEntry DShootingMode = shooterTab.add("Shooting Mode",4).withPosition(1, 5).getEntry();
  private NetworkTableEntry DDistance = shooterTab.add("Distance to goal", 0.0).withPosition(2,0).getEntry();
  private NetworkTableEntry DShooterRPM = shooterTab.add("Shooter RPM", 0.0).withPosition(2,1).getEntry();
  private NetworkTableEntry BCargoRunning = shooterTab.add("Is the CDS Running",false).withPosition(2, 2).getEntry();
  private NetworkTableEntry DShooterRPMInput = shooterTab.add("Shooter RPM Input",2000).withPosition(2, 3).getEntry();
  private NetworkTableEntry IDelayTable = shooterTab.add("Current I delay",0).withPosition(2, 4).getEntry(); //Delay before the CDS deliver the ball for the PID to stablize the speed
  private double MaxOutputConstant;
  private double MinOutputConstant;
  private ShooterConfig[] DistanceArray;




  
  
  //private NetworkTableEntry PID_F = shooterTab.addPersistent("PID F", Constants.Shooter.).withPosition(3, 2).getEntry();
  //private NetworkTableEntry PID_Izone = shooterTab.addPersistent("PID I Range", Constants.Shooter.).withPosition(4, 2).getEntry();
  //private NetworkTableEntry PID_MaxOutput = shooterTab.addPersistent("PID Peak Output", Constants.Shooter.d).withPosition(5, 2).getEntry();








  public ShooterSubsystem() {
    //SmartDashboard.putNumber("RPMIN", RPMIN);
    aimMode = 4;
    cargo_motorController = new MotorController("Shooter Cargo", Constants.Shooter.shooterCargoID,40,true);
    kCargoController = cargo_motorController.getPID();
    kCargoEncoder = cargo_motorController.getEncoder();
    shooter_motorController = new MotorController("Shooter", Constants.Shooter.shooterID, 40, true);
    KShooterController = shooter_motorController.getPID();
    KShooterEncoder = shooter_motorController.getEncoder();
    
    DistanceArray = new ShooterConfig[3];

    
    DistanceArray[0] = new ShooterConfig(5,64,2263);
    DistanceArray[1] = new ShooterConfig(10,80,3065);
    DistanceArray[2] = new ShooterConfig(15,82,3420);

   
    //TODO:FIll lookup table



    



    KShooterController.setIMaxAccum(0.9, 0);
    //TODO: Set it up when the hood is avaliable
    //hood_motorController = new MotorController("Hood", Constants.Shooter.hoodID,40,true);
    //KHoodController = hood_motorController.getPID();
    //KHoodEncoder = shooter_motorController.getEncoder();

  }
  public void updatePID(){

    
    if (dashTunePid.getBoolean(false)){
      
      Pconstant = PID_P.getDouble(0);
      Iconstant = PID_I.getDouble(0);
      Dconstant = PID_D.getDouble(0);
      Fconstant = PID_F.getDouble(0);
      MaxOutputConstant = PID_MaxOutput.getDouble(0);
      MinOutputConstant = PID_MinOutput.getDouble(0);
      IMaxAccumconstant = PID_IMaxAccum.getDouble(0);
      IMaxAccumIDconstant =(int)PID_IMaxAccumID.getDouble(0);
      shooter_motorController.getPID().setOutputRange(MaxOutputConstant, MinOutputConstant);
      shooter_motorController.getPID().setP(Pconstant);
      shooter_motorController.getPID().setI(Iconstant);
      shooter_motorController.getPID().setD(Dconstant);
      shooter_motorController.getPID().setFF(Fconstant);
      shooter_motorController.getPID().setIMaxAccum(IMaxAccumconstant, IMaxAccumIDconstant);
    }
  }
  public double[] lookup(double Currentdistance){
    if (Currentdistance < DistanceArray[0].getDistance()){
      return DistanceArray[0].getVelocityAndAngle();
    }
    for(int i=1;i<DistanceArray.length;i++){
      if (Currentdistance >= DistanceArray[i].getDistance()){
        return ShooterConfig.interpolate(DistanceArray[i-1], DistanceArray[i], Currentdistance);
      }
      
    }
    return DistanceArray[DistanceArray.length-1].getVelocityAndAngle();
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
    KShooterController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }
      //KShooterController.setIAccum(0);

  }
  public void  SetCargoBoolean(boolean a){
    BCargoRunning.setBoolean(a);
  }
  public void UpdateIdelay(double i){
    IDelayTable.setDouble(i);
  }

  public void runCargo(boolean a,boolean reversed) {
    if(a){
      if(reversed){cargo_motorController.setSpeed(-0.2);}
      else{cargo_motorController.setSpeed(0.2);}
    }else{
      cargo_motorController.setSpeed(0.0);
    }

  }

  public boolean wheelReady(){
    double flywheelSpeed = KShooterEncoder.getVelocity();
    
    return (flywheelSpeed > currentRPM - 15 && flywheelSpeed < currentRPM + 15);
  }
  public void setAimMode(Double m) {
    aimMode = m;
    DShootingMode.setDouble(aimMode);

  }

  public void cycleAimModeUp() {
    aimMode ++;
    if (aimMode>3){
      aimMode = 0;
    }
    DShootingMode.setDouble(aimMode);
  }

  public void cycleAimModeDown() {
    aimMode--;
    if (aimMode < 0) {
      aimMode = 3;
    }
    DShootingMode.setDouble(aimMode);
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
    currentRPM = (int) (Math.ceil(ProjectilePrediction(Constants.Shooter.shooterHeight, 0, Constants.Shooter.highHeight,getDistance(), 32, Constants.Shooter.airboneTime)[0]));
  }
  public void lookuptablemode(double distance){
    double [] returnarray = lookup(distance);
    windFlywheel(returnarray[0]);
    //adjusthood(returnarray[1]);
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
    // Check what aimMode is active, gets distance if AUTO, winds flywheel, adjusts
    // hood correspondingly
    
    switch ((int)aimMode) {
      case 0: // Case for LOW mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.Shooter.LOWAngle);
        windFlywheel(Constants.Shooter.LOWRPM);
        currentRPM = Constants.Shooter.LOWRPM;
        break;
      case 1: // Case for AUTO mode, calculates trajectory and winds flywheel/adjusts hood to
              // a dynamic state
              automode();
       

        break;
      case 2: // Case for LAUNCH mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.Shooter.LAUNCHAngle);
        windFlywheel(Constants.Shooter.LAUNCHRPM);
        currentRPM = Constants.Shooter.LAUNCHRPM;
        break;
      case 3: // Case for TARMAC mode, winds flywheel to preset RPM and adjusts hood to preset
              // angle
        adjustHood(Constants.Shooter.TARMACAngle);
        windFlywheel(Constants.Shooter.TARMACRPM);
        currentRPM = Constants.Shooter.TARMACRPM;
        break;
      case 4: //Case for TEST mode, just takes an RPM and winds
        windFlywheel(DShooterRPMInput.getDouble(0));
        currentRPM = DShooterRPMInput.getDouble(0);
        break;
      case 5:
        windFlywheel(lookup(getDistance())[0]);
        currentRPM = lookup(getDistance())[0];

        break;
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("IAccum",KShooterController.getIAccum());
    //SmartDashboard.putNumber("dist", getDistance());
    if(DShooterRPM.getDouble(0.0) != shooter_motorController.getEncoder().getVelocity()){
      DShooterRPM.setDouble(shooter_motorController.getEncoder().getVelocity());
    }
    if(DDistance.getDouble(0.0)!= getDistance()){
      DDistance.setDouble(getDistance());
    }
    if (dashTunePid.getBoolean(false)){
      if ((shooter_motorController.getPID().getP() != PID_P.getDouble(0))||(shooter_motorController.getPID().getI() != PID_I.getDouble(0)) ||(shooter_motorController.getPID().getD() != PID_D.getDouble(0))) {
        updatePID();
      }
     
      //dashTunePid.setBoolean(false);

    }
    if (ShooterReverted.getBoolean(false)!= shooter_motorController.getEncoder().getInverted()){
      shooter_motorController.setInverted(ShooterReverted.getBoolean(false));
    }
    if (DShootingMode.getDouble(0) != aimMode){
      aimMode = DShootingMode.getDouble(0);
    }

  }

}
