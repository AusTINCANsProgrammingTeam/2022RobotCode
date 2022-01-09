

public class MotorController {
    
    private String mName;
    private CANSparkMax mSparkMax;
    private CANEncoder mEncoder;
    private CANPIDController mPIDController = null;
    
    // PID
    private double mP;
    private double mI;
    private double mD;
}
