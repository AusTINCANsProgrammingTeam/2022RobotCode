package frc.robot.common.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.MotorDefaults;

public class MotorController {
    public static enum MotorConfig {
        //Drivebase
        driveLeftFront(13,50),
        driveLeftRear(14,50),
        driveRightFront(6,50,true),
        driveRightRear(7,50,true),
        //Intake
        intakeMotor(1,true),
        intakeDeploy(8),
        //CDS
        CDSBelt(3,true),
        singulatorOne(2,true),
        singulatorTwo(9),
        //Shooter
        shooterOne(10),
        shooterTwo(11),
        //Stopper
        stopperWheel(4),
        //Climb
        climbArmOne(5,10),
        climbArmTwo(12,10,true),
        climbPoleOne(15,10),
        climbPoleTwo(16,10,true);

        private int ID;
        private int currentLimit;
        private double openLoopRampRate;
        private boolean reversed;

        MotorConfig(int ID, int currentLimit, double openLoopRampRate, boolean reversed){
            this.ID = ID;
            this.currentLimit = currentLimit;
            this.openLoopRampRate = openLoopRampRate;
            this.reversed = reversed;
        }

        MotorConfig(int ID, int currentLimit, double openLoopRampRate){
            this(ID, currentLimit, openLoopRampRate, false);
        }

        MotorConfig(int ID, int currentLimit, boolean reversed){
            this(ID, currentLimit, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID, int currentLimit){
            this(ID, currentLimit, MotorDefaults.kOpenLoopRampRate, false);
        }

        MotorConfig(int ID, boolean reversed){
            this(ID, MotorDefaults.kCurrentLimit, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID){
            this(ID, MotorDefaults.kCurrentLimit, MotorDefaults.kOpenLoopRampRate, false);
        }

        public int getID(){
            return ID;
        }

        public int getCurrentLimit(){
            return currentLimit;
        }

        public double getOpenLoopRampRate(){
            return openLoopRampRate;
        }

        public boolean getReversed(){
            return reversed;
        }
    }

    public static CANSparkMax constructMotor(MotorConfig config){
        CANSparkMax motor = new CANSparkMax(config.getID(), MotorType.kBrushless);
        motor.setSmartCurrentLimit(config.getCurrentLimit());
        motor.setOpenLoopRampRate(motor.getOpenLoopRampRate());
        motor.setInverted(motor.getInverted());
        return motor;
    }

    public static SparkMaxPIDController constructPIDController(CANSparkMax motor, double[] PIDArray){
      SparkMaxPIDController PIDController = motor.getPIDController();
      PIDController.setP(PIDArray[0]);
      PIDController.setI(PIDArray[1]);
      PIDController.setD(PIDArray[2]);
      return PIDController;
    }
}