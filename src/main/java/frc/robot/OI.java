package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {
    //Operator Interface (OI) class containing all control information
    private static final int kDriverJoystickPort = 0;
    private static final int kOperatorJoystickPort = 1;

    public static final class Driver{
        private static final Joystick kJoystick = new Joystick(OI.kDriverJoystickPort);

        private static final int kIntakeButtonID = 8; //Run the CDS and intake forwards
        private static final int kOuttakeButtonID = 6; //Run the CDS and intake backwards
        private static final int kShootButtonID = 7; //Fire the shooter

        private static final int kDriveSpeedAxis = 1; //This axis is inverted
        private static final int kDriveRotationAxis = 2;

        private static final ControlCurve kDriveSpeedCurve = new ControlCurve(0.9,0,0,0);
        private static final ControlCurve kDriveRotationCurve = new ControlCurve(0.85,0,0,0);

        public static Supplier<Double> getDriveSpeedSupplier(){
            return () -> kDriveSpeedCurve.calculate(-kJoystick.getRawAxis(kDriveSpeedAxis));
        }

        public static Supplier<Double> getDriveRotationSupplier(){
            return () -> kDriveRotationCurve.calculate(kJoystick.getRawAxis(kDriveRotationAxis));
        }

        public static JoystickButton getIntakeButton(){
            return new JoystickButton(kJoystick, kIntakeButtonID);
        }

        public static JoystickButton getOuttakeButton(){
            return new JoystickButton(kJoystick, kOuttakeButtonID);
        }

        public static JoystickButton getShootButton(){
            return new JoystickButton(kJoystick, kShootButtonID);
        }
    }

    public static final class Operator{
        private static final Joystick kJoystick = new Joystick(OI.kOperatorJoystickPort);

        private static final int kEnableClimbButtonID = 10; //Toggle the ability to manipulate climb arms
        private static final int kDeployClimbButtonID = 1; //Automatically deploy climb arms
        private static final int kCDSForwardButtonID = 8; //Run the CDS forward
        private static final int kOuttakeButtonID = 6; //Run the CDS and intake backwards

        private static final int kClimbArmAxis = 1; //This axis is inverted
        private static final int kClimbPoleAxis = 3; //This axis is inverted

        private static final ControlCurve kClimbArmCurve = new ControlCurve(1,0,0,0.1);
        private static final ControlCurve kClimbPoleCurve = new ControlCurve(1,0,0,0.1);

        public static Supplier<Double> getClimbArmSupplier(){
            return () -> kClimbArmCurve.calculate(-kJoystick.getRawAxis(kClimbArmAxis));
        }

        public static Supplier<Double> getClimbPoleSupplier(){
            return () -> kClimbPoleCurve.calculate(-kJoystick.getRawAxis(kClimbPoleAxis));
        }
        
        public static JoystickButton getEnableClimbButton(){
            return new JoystickButton(kJoystick, kEnableClimbButtonID);
        }

        public static JoystickButton getDeployClimbButton(){
            return new JoystickButton(kJoystick, kDeployClimbButtonID);
        }

        public static JoystickButton getCDSForwardButton(){
            return new JoystickButton(kJoystick, kCDSForwardButtonID);
        }

        public static JoystickButton getOuttakeButton(){
            return new JoystickButton(kJoystick, kOuttakeButtonID);
        }
    }

    public static class ControlCurve{
        private double ySaturation; //Maximum output, in percentage of possible output
        private double yIntercept; //Minimum output, in percentage of saturation
        private double curvature; //Curvature shift between linear and cubic
        private double deadzone; //Range of input that will always return zero output

        public ControlCurve(double ySaturation, double yIntercept, double curvature, double deadzone){
            this.ySaturation = ySaturation;
            this.yIntercept = yIntercept;
            this.curvature = curvature;
            this.deadzone = deadzone;
        }

        public double calculate(double input){
            /* Two equations, separated by a ternary
            The first is the deadzone
            y = 0 {|x| < d}
            The second is the curve
            y = a(sign(x) * b + (1 - b) * (c * x^3 + (1 - c) * x)) {|x| >= d}
            Where
            x = input
            y = output
            a = ySaturation
            b = yIntercept
            c = curvature
            d = deadzone
            and 0 <= a,b,c,d < 1 
            */
            return Math.abs(input) <  deadzone ? 0 : 
            ySaturation * (Math.signum(input) * yIntercept + 
            (1 - yIntercept) * (curvature * Math.pow(input, 3) +
            (1 - curvature) * input));
        }
    }
}