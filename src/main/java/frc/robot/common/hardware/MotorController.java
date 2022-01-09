package frc.robot.common.hardware;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MotorController {
    
    private String mName;
    private CANSparkMax mSparkMax;
    private CANEncoder mEncoder;
    private CANPIDController mPIDController = null;
    
    // PID
    private double mP;
    private double mI;
    private double mD;

    public MotorController(String name, int deviceID) {
        mName = name;
        mSparkMax = new CANSparkMax(deviceID, MotorType.kBrushless);

        mEncoder = mSparkMax.getEncoder();
        mSparkMax.restoreFactoryDefaults();
    }

    public MotorController(String name, int deviceID, int smartCurrentLimit, boolean... enablePid) {
        this(name, deviceID); // intializes CANSparkMax and Encoder
        mSparkMax.setSmartCurrentLimit(smartCurrentLimit);
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

    

}
