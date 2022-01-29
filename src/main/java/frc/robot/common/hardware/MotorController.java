package frc.robot.common.hardware;

import javax.management.JMRuntimeException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import netscape.javascript.JSException;

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
        mSparkMax.setSmartCurrentLimit(40); // default current limit is 40A

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
        // Check first that mSparkMax has been instantiated
        if(mSparkMax == null) {
            throw new NullPointerException("Spark MAX motor has not been instantiated.");
        }
        return mSparkMax;
    }

    public RelativeEncoder getEncoder() {
        // Check first that mEncoder has been instantiated
        if(mEncoder == null) {
            throw new NullPointerException("Encoder has not been instantiated.");
        }
        return mEncoder;
    }

    public SparkMaxPIDController getPID() { 
        // Check first that mPIDController has been instantiated
        if(mPIDController == null) {
            throw new NullPointerException("PID Controller has not been instantiated.");
        }
        return mPIDController;
    }

    // sets speed of motor
    public void setSpeed(double speed) {
        mSparkMax.set(speed);
    }

    // set follow
    public void setFollow(MotorController m) {
        mSparkMax.follow(m.getSparkMax());
    }

    // set inverted
    public void setInverted(boolean b) {
        mSparkMax.setInverted(b);
    }

    // get speeds of wheel side
    public double getSpeed() {
        return mEncoder.getVelocity();
    }

    // get boolean for whether if it's inverted
    public boolean isInverted() {
        return mSparkMax.getInverted();
    }

    public void setPID() {
        // Check first that mPIDController has been instantiated
        if(mPIDController == null) {
            throw new NullPointerException("PID Controller has not been instantiated.");
        }

        mPIDController.setP(mP);
        mPIDController.setI(mI);
        mPIDController.setD(mD);
        SmartDashboard.putNumber(mName+" P Value", mP);
        SmartDashboard.putNumber(mName+" I Value", mI);
        SmartDashboard.putNumber(mName+" D Value", mD);
    }

    // Updates the Smart Dashboard and checks the PID values to determine if update is needed
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
    }

    // Mode can be coast or brake
    public void setIdleMode(CANSparkMax.IdleMode mode) {
        mSparkMax.setIdleMode(mode);    
    }

}
