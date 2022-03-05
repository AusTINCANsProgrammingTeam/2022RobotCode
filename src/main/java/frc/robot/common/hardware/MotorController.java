package frc.robot.common.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class MotorController extends CANSparkMax {

  private String mName;
  private RelativeEncoder mEncoder;
  private SparkMaxPIDController mPIDController;

  // PID
  private double mP;
  private double mI;
  private double mD;

  private double mFF; // feedforward value

  // default constructor
  public MotorController(String name, int deviceID) {
    super(deviceID, MotorType.kBrushless);
    mName = name;
    restoreFactoryDefaults();

    // Create default values for Spark Max motor controller
    setSmartCurrentLimit(Constants.defaultCurrentLimit); // default current limit is 40A
    setIdleMode(CANSparkMax.IdleMode.kCoast);            // default mode is Coast
    setOpenLoopRampRate(Constants.openLoopRampRate);     // default open loop rate

    mPIDController = getPIDController();
    mEncoder = getEncoder();
  }

  public MotorController(String name, int deviceID, double[] PID) {
    this(name, deviceID); // intializes CANSparkMax, Encoder, and PIDController
    setPID(PID);
  
    setSmartCurrentLimit(40); // default current limit is 40A
    setIdleMode(CANSparkMax.IdleMode.kCoast); // default mode is Coast
    mPIDController = getPIDController();
    mEncoder = getEncoder();
  }

  public void setPID(double[] PID) {
    mP = PID[0];
    mI = PID[1];
    mD = PID[2];
    activatePID(PID);
  }

  public RelativeEncoder getEncoder() {
    // Check first that mEncoder has been instantiated
    if (mEncoder == null) {
      throw new NullPointerException("Encoder for " + this.mName + " has not been instantiated.");
    }
    return mEncoder;
  }

  public SparkMaxPIDController getPIDCtrl() {
    // Check first that mPIDController has been instantiated
    if (mPIDController == null) {
      throw new NullPointerException(
          "PID Controller for " + this.mName + " has not been instantiated.");
    }
    return mPIDController;
  }

  // sets speed of motor
  public void setSpeed(double speed) {
    set(speed);
  }

  public String getName() {
    return this.mName;
  }

  // get speeds of wheel side
  public double getSpeed() {
    // Check first that mEncoder has been instantiated
    if (mEncoder == null) {
      throw new NullPointerException("Encoder for " + this.mName + " has not been instantiated.");
    }
    return mEncoder.getVelocity();
  }

  public void activatePID(double[] PID) {
    // Check first that mPIDController has been instantiated
    if (mPIDController == null) {
      throw new NullPointerException(
          "PID Controller for " + this.mName + " has not been instantiated.");
    }

    mPIDController.setP(mP);
    mPIDController.setI(mI);
    mPIDController.setD(mD);

    SmartDashboard.putNumberArray(mName, PID);
  }

  // Updates the Smart Dashboard and checks the PID values to determine if update is needed
  public void updateSmartDashboard() {
    // The simulation crashes whenever .getEncoder() is called
    if (mPIDController != null) {
      // if (SmartDashboard.getNumber(mName + " P Value", mP) != mP) {
      //   mP = SmartDashboard.getNumber(mName + " P Value", mP);
      //   mPIDController.setP(mP);
      // }
      // if (SmartDashboard.getNumber(mName + " I Value", mI) != mI) {
      //   mI = SmartDashboard.getNumber(mName + " I Value", mI);
      //   mPIDController.setI(mI);
      // }
      // if (SmartDashboard.getNumber(mName + " D Value", mD) != mD) {
      //   mD = SmartDashboard.getNumber(mName + " D Value", mD);
      //   mPIDController.setD(mD);
      // }
      // if (SmartDashboard.getNumber(mName + " FF Value", mFF) != mFF) {
      //   mFF = SmartDashboard.getNumber(mName + " FF Value", mFF);
      //   mPIDController.setFF(mFF);
      // }



    }
  }
}
