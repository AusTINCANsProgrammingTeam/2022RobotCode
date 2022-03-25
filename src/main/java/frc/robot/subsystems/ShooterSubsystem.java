// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AimModes;
import frc.robot.common.hardware.MotorController;

public class ShooterSubsystem extends SubsystemBase {

  private MotorController flywheelController;
  private MotorController flywheel2Controller;
  private SimpleMotorFeedforward flywheelFF;
  private SparkMaxPIDController flywheelPID;
  private PIDController flywheelWPID;
  private RelativeEncoder flywheelEncoder;
  private MotorController hoodController;
  private SparkMaxPIDController hoodPID;
  private RelativeEncoder hoodEncoder;
  private MotorController stopperController;

  private AimModes aimMode;
  private double targetRPM;
  private double currentRPM;
  private double smoothRPM;

  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry DTRPM = operatorTab.add("T-RPM", 0).withPosition(0, 2).getEntry();
  private NetworkTableEntry DRPM =
      operatorTab.add("RPM", 0).withWidget(BuiltInWidgets.kDial).withSize(2, 2).getEntry();
  private NetworkTableEntry BCargoRunning =
      operatorTab.add("Flywheel Ready", false).withPosition(2, 0).getEntry();
  private NetworkTableEntry SAimMode =
      operatorTab.add("Aim Mode", "TEST").withPosition(2, 1).getEntry();
  private NetworkTableEntry BOverride =
      operatorTab
          .add("Override", false)
          .withPosition(1, 2)
          .withWidget(BuiltInWidgets.kToggleSwitch)
          .getEntry();

  private ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter Tab");
  private NetworkTableEntry PID_P;
  private NetworkTableEntry PID_I;
  private NetworkTableEntry PID_D;
  private NetworkTableEntry PID_F;
  private NetworkTableEntry DSmoothRPM;

  private ShooterConfig[] DistanceArray;

  public ShooterSubsystem() {
    if (Constants.DebugMode) {
      instantiateDebugTab();
    }
    smoothRPM = 0;
    aimMode = AimModes.TEST;
    // Initializes the SparkMAX for the flywheel motors
    flywheelController =
        new MotorController("Flywheel", Constants.Shooter.shooterID, Constants.Shooter.kPIDFArray);
    flywheel2Controller = new MotorController("Flywheel 2", Constants.Shooter.shooter2ID);
    flywheelFF =
        new SimpleMotorFeedforward(
            Constants.Shooter.kSg, Constants.Shooter.kVg, Constants.Shooter.kAg);
    flywheelPID = flywheelController.getPIDCtrl();
    flywheelEncoder = flywheelController.getEncoder();
    flywheelController.enableVoltageCompensation(11);
    flywheel2Controller.enableVoltageCompensation(11);
    flywheel2Controller.follow(flywheelController, true);

    // Initializes the SparkMAX for the hood TODO: Set this up when possible
    /*hoodController = new  MotorController("Hood", Constants.hoodID);
    hoodPID = hoodController.getPID();
    hoodEncoder = hoodController.getEncoder();*/
    // Initializes the SparkMAX for the cargo stopper
    stopperController = new MotorController("Shooter Cargo", Constants.Shooter.shooterCargoID);
    // Initializes PID for the hood TODO: Set this up when possible
    /*hoodPID.setP(0.0);
    hoodPID.setI(0.0);
    hoodPID.setD(0.0);
    hoodPID.setOutputRange(0, 1);*/
    // Initializes Additional PID for the shooter
    flywheelPID.setIMaxAccum(Constants.Shooter.kMaxIAccum, Constants.Shooter.kMaxISlot);
    flywheelPID.setOutputRange(Constants.Shooter.kMinOutput, Constants.Shooter.kMaxOutput);
    // flywheelPID.setFF(Constants.Shooter.kF);

    DistanceArray = new ShooterConfig[3];
    DistanceArray[0] = new ShooterConfig(5, 64, 2263);
    DistanceArray[1] = new ShooterConfig(10, 80, 3065);
    DistanceArray[2] = new ShooterConfig(15, 82, 3420);
    // TODO:FIll lookup table
  }

  private void instantiateDebugTab() {
    shooterTab = Shuffleboard.getTab("Shooter Tab");
    PID_P = shooterTab.add("PID P", Constants.Shooter.kPIDFArray[0]).withPosition(0, 1).getEntry();
    PID_I = shooterTab.add("PID I", Constants.Shooter.kPIDFArray[1]).withPosition(0, 2).getEntry();
    PID_D = shooterTab.add("PID D", Constants.Shooter.kPIDFArray[2]).withPosition(0, 3).getEntry();
    PID_F = shooterTab.add("PID F", Constants.Shooter.kF).withPosition(0, 4).getEntry();
    DSmoothRPM = shooterTab.add("Smooth RPM", 0.0).getEntry();
  }

  public void updatePID() {
    if (Constants.DebugMode) {
      flywheelPID.setP(PID_P.getDouble(0));
      flywheelPID.setI(PID_I.getDouble(0));
      flywheelPID.setD(PID_D.getDouble(0));
      flywheelPID.setFF(PID_F.getDouble(0));
    }
  }

  public void adjustHood(double a) {
    // Adjusts Hood using PID control to passed angle a
    // hoodPID.setReference(a, CANSparkMax.ControlType.kPosition); TODO: Set this up when possible
  }

  public double[] lookup(double Currentdistance) {
    if (Currentdistance < DistanceArray[0].getDistance()) {
      return DistanceArray[0].getVelocityAndAngle();
    }
    for (int i = 1; i < DistanceArray.length; i++) {
      if (Currentdistance >= DistanceArray[i].getDistance()) {
        return ShooterConfig.interpolate(DistanceArray[i - 1], DistanceArray[i], Currentdistance);
      }
    }
    return DistanceArray[DistanceArray.length - 1].getVelocityAndAngle();
  }

  public void windFlywheel(double rpm) {

    // Winds Flywheel using PID control to passed rpm
    if (rpm == 0) {
      flywheelPID.setReference(0, CANSparkMax.ControlType.kVoltage);
    } else {
      DTRPM.setDouble(rpm);
      targetRPM = rpm;
      flywheelPID.setReference(
      rpm,
      CANSparkMax.ControlType.kVelocity,
      Constants.Shooter.kMaxISlot,
      flywheelFF.calculate(rpm / 62.0));
    }
  }

  public void resetIAccum() {
    if (flywheelEncoder.getVelocity() < 500) {
      flywheelPID.setIAccum(0);
    }
  }

  public void setCargoBoolean(boolean a) {
    BCargoRunning.setBoolean(a);
  }

  public void runCargo(double speed) {
    stopperController.set(speed);
  }

  public boolean wheelReady() {
    return (smoothRPM > targetRPM - 56 && smoothRPM < targetRPM + 56);
  }

  public void setAimMode(AimModes a) {
    aimMode = a;
    SAimMode.setString(aimMode.toString());
  }

  public AimModes getAimMode() {
    return aimMode;
  }

  public void cycleAimModeNext() {
    aimMode.next();
    SAimMode.setString(aimMode.toString());
  }

  public void cycleAimModePrevious() {
    aimMode.previous();
    SAimMode.setString(aimMode.toString());
  }

  public double getTY() {
    // Returns TY, the vertical angle of the target from the limelight
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getDistance() {
    // Uses Limelight to find distance to High Goal
    return (Constants.Shooter.highHeight - Constants.Shooter.LLHeight)
        / Math.tan(Math.toRadians((getTY() + Constants.Shooter.LLAngle))); // Return distance in ft
  }

  public void prime() {
    // Check what aimMode is active, gets distance if AUTO, winds flywheel, adjusts
    // hood correspondingly
    if (BOverride.getBoolean(false)) {
      windFlywheel(DTRPM.getDouble(0));
      SAimMode.setString("OVERRIDE");
    } else {
      SAimMode.setString(aimMode.toString());
      switch (aimMode) {
        case EJECT: // aimMode used to eject unwanted balls from the shooter
        case LOW: // aimMode used to dump into the low goal from ~1ft
        case TARMAC: // aimMode used to shoot into the high goal from ~2ft
        case LAUNCH: // aimMode used to shoot into the high goal from the launchpad
          adjustHood(aimMode.getAngle());
          windFlywheel(aimMode.getRPM());
          break;
        case AUTO: // aimMode used to automatically shoot into the high goal
          windFlywheel(lookup(getDistance())[0]);
          // TODO: Also adjust hood here
          break;
        case TEST: // aimMode to take a RPM from the dashboard
          windFlywheel(DTRPM.getDouble(0));
          break;
      }
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ff", flywheelFF.calculate(2600 / 60));
    // This method will be called once per scheduler run
    currentRPM = flywheelEncoder.getVelocity();
    smoothRPM = Constants.Shooter.kA * currentRPM + smoothRPM * (1 - Constants.Shooter.kA);

    DRPM.setDouble(currentRPM);

    if (Constants.DebugMode) {
      if ((flywheelPID.getP() != PID_P.getDouble(0))
          || (flywheelPID.getI() != PID_I.getDouble(0))
          || (flywheelPID.getD() != PID_D.getDouble(0))
          || (flywheelPID.getFF() != PID_F.getDouble(0))) {
        updatePID();
      }
    }
  }
}
