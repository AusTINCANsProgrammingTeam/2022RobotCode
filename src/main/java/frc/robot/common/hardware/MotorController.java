package frc.robot.common.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorController {
    
    private String mName;
    private CANSparkMax mSparkMax;
    private RelativeEncoder mEncoder;            
    private SparkMaxPIDController mPIDController;
    
    // PID
    private double mP;
    private double mI;
    private double mD;

    // default constructor
    public MotorController(String name, int deviceID) {
        mName = name;
        mSparkMax = new CANSparkMax(deviceID, MotorType.kBrushless);
        mPIDController = mSparkMax.getPIDController();
        setPID();
        mEncoder = mSparkMax.getEncoder();
        mSparkMax.restoreFactoryDefaults();

    }


    public MotorController(String name, int deviceID, int smartCurrentLimit, boolean... enablePid) {
        this(name, deviceID);                               // intializes CANSparkMax and Encoder
        mSparkMax.setSmartCurrentLimit(smartCurrentLimit);  // set smartCurrentLimit

        // If enablePid has any number of booleans greater than 0 we are enabling pid
        if (enablePid.length > 0)
        {
            mP = SmartDashboard.getNumber(mName + " P Value", 1.0);
            mI = SmartDashboard.getNumber(mName + " I Value", 0.0);
            mD = SmartDashboard.getNumber(mName + " D Value", 0.0);
            mPIDController = mSparkMax.getPIDController();
            setPID();
        }
        mSparkMax.setOpenLoopRampRate(.1);
    }

    public CANSparkMax getSparkMax() {
        return mSparkMax;
    }

    public RelativeEncoder getEncoder() {
        return mEncoder;
    }

    public SparkMaxPIDController getPID() {
        return mPIDController;
    }

    public void setPID() {
        mPIDController.setP(mP);
        mPIDController.setI(mI);
        mPIDController.setD(mD);
        SmartDashboard.putNumber(mName+" P Value", mP);
        SmartDashboard.putNumber(mName+" I Value", mI);
        SmartDashboard.putNumber(mName+" D Value", mD);
    }
    public void setSpeed(double speed) {
        mSparkMax.set(speed);
    }

  /*  // Updates the Smart Dashboard and checks the PID values to determine if update is needed
    public void updateSmartDashboard() {
        // The simulation crashes whenever .getEncoder() is called
        if(mPIDController != null) {
            if (SmartDashboard.getNumber(mName + " P Value", mP) != mP) {
                mP = SmartDashboard.getNumber(mName + " P Value", mP);
                mPIDController.setP(mP);
            }
            if (SmartDashboard.getNumber(mName + " I Value", mI) != mI) {
                mI = SmartDashboard.getNumber(mName + " I Value", mI);
                mPIDController.setI(mI);
            }
            if (SmartDashboard.getNumber(mName + " D Value", mD) != mD) {
                mD = SmartDashboard.getNumber(mName + " D Value", mD);
                mPIDController.setD(mD);
            }
        }
    }*/

}
