package frc.robot.common.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


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

        mEncoder = mSparkMax.getEncoder();
        mSparkMax.restoreFactoryDefaults();
    }


    public MotorController(String name, int deviceID, int smartCurrentLimit, boolean... enablePid) {
        this(name, deviceID);                               // intializes CANSparkMax and Encoder
        mSparkMax.setSmartCurrentLimit(smartCurrentLimit);  // set smartCurrentLimit

        // If enablePid has any number of booleans greater than 0 we are enabling pid
        if (enablePid.length > 0)
        {
            mP = 1;
            mI = 0;
            mD = 0;
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
       
    }

    // Updates the Smart Dashboard and checks the PID values to determine if update is needed
    public void updateSmartDashboard() {

    }

}
