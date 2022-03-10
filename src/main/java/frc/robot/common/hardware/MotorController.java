package frc.robot.common.hardware;

import com.revrobotics.CANSparkMax;
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
  private double mFF;

  // default constructor
  public MotorController(String name, int deviceID) {
    super(deviceID, MotorType.kBrushless);
    mName = name;
    restoreFactoryDefaults();

    // Create default values for Spark Max motor controller
    setSmartCurrentLimit(Constants.defaultCurrentLimit); // default current limit is 40A
    setIdleMode(CANSparkMax.IdleMode.kCoast); // default mode is Coast
    setOpenLoopRampRate(Constants.openLoopRampRate); // default open loop rate

    mPIDController = getPIDController();
    mEncoder = getEncoder();
  }

  // Set PID option for motor controller
  public MotorController(String name, int deviceID, double[] PID) {
    this(name, deviceID); // intializes CANSparkMax, Encoder, and PIDController
    setPID(PID);
  }

  public void setPID(double[] PID) {
    mP = PID[0];
    mI = PID[1];
    mD = PID[2];
    mFF = PID[3];
    activatePID(PID);
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

  private void activatePID(double[] PID) {
    // Check first that mPIDController has been instantiated
    if (mPIDController == null) {
      throw new NullPointerException(
          "PID Controller for " + this.mName + " has not been instantiated.");
    }

    mPIDController.setP(mP);
    mPIDController.setI(mI);
    mPIDController.setD(mD);
    mPIDController.setFF(mFF);

    // add the values to smart dashboard
    SmartDashboard.putNumber(mName + " P value", mP);
    SmartDashboard.putNumber(mName + " I value", mI);
    SmartDashboard.putNumber(mName + " D value", mD);

    updateSmartDashboard(); // post onto smart dashboard
  }

  // Updates the Smart Dashboard and checks the PID values to determine if update is needed
  public void updateSmartDashboard() {
    if (mPIDController != null) {
      // TODO: having a sole shuffleboard tab for PID tuning might be beneficical, there's a built
      // in widget especially for PID

      // check if values were updated in smart dashboard
      // nothing in it at the start, assign it to value passed in earlier
      double currentP = SmartDashboard.getNumber(mName + " P value", mP);
      if (currentP != mP) {
        mP = currentP;
        mPIDController.setP(mP);
      }

      double currentI = SmartDashboard.getNumber(mName + " I value", mI);
      if (currentI != mI) {
        mI = currentI;
        mPIDController.setI(mI);
      }

      double currentD = SmartDashboard.getNumber(mName + " D value", mD);
      if (currentD != mD) {
        mD = currentD;
        mPIDController.setD(mD);
      }

      double currentFF = SmartDashboard.getNumber(mName + " FF value", mFF);
      if (currentFF != mFF) {
        mFF = currentFF;
        mPIDController.setFF(mFF);
      }
    }
  }
}
