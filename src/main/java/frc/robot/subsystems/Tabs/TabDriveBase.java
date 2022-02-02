package frc.robot.subsystems.Tabs;

import frc.robot.subsystems.DriveBaseSubsystem;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class TabDriveBase {

    // each box on the shuffleboard, sb stands for shuffleboard
    private NetworkTableEntry sbLeftWheelSpeed;
    private NetworkTableEntry sbRightWheelSpeed;
    private NetworkTableEntry sbGyroAngle;
    private NetworkTableEntry sbVelocityConversionFactor;

    // TODO: other indicators for motor controllers: if inverted, what motors they're following, + other factors important to driver and debugging

    // DriveBaseSubsystem object
    private DriveBaseSubsystem mDriveBaseSubsystem;

    public TabDriveBase(DriveBaseSubsystem d) {
        this.mDriveBaseSubsystem = d;

        // gets the DriveBase tab, if it doesn't exist, create it with the name "DriveBase"
        ShuffleboardTab dtTab = Shuffleboard.getTab("DriveBase");

        sbLeftWheelSpeed = dtTab.add("Left Wheel Speed", 0).withSize(2, 2).withPosition(0, 0).getEntry();
        sbRightWheelSpeed = dtTab.add("Right Wheel Speed", 0).withSize(2, 2).withPosition(3, 0).getEntry();
        sbGyroAngle = dtTab.add("Gyro Angle", 0).withSize(1, 1).withPosition(2, 0).getEntry();
        sbVelocityConversionFactor = dtTab.add("Velocity Conversion Factor from Encoder", 0).withSize(2,2).withPosition(0,1).getEntry();
        
    }

    public void periodic() {
        // constantly update the board
        
        double leftSpeed = mDriveBaseSubsystem.getLeftSpeed();
        double rightSpeed = mDriveBaseSubsystem.getRightSpeed();
        double gyroAngle = mDriveBaseSubsystem.getGyroAngle();
        double velocityConversionFactor = mDriveBaseSubsystem.getVelocityConversionFactor();

        sbLeftWheelSpeed.setDouble(leftSpeed);
        sbRightWheelSpeed.setDouble(rightSpeed);
        sbGyroAngle.setDouble(gyroAngle);
        sbVelocityConversionFactor.setDouble(velocityConversionFactor);
        
    }
}
