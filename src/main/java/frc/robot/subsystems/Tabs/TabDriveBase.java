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
    private NetworkTableEntry sbLeftPosition;
    private NetworkTableEntry sbRightPosition;

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
        sbVelocityConversionFactor = dtTab.add("Velocity Conversion Factor from Encoder", 0).withSize(1,1).withPosition(0,3).getEntry();
        sbLeftPosition = dtTab.add("Left Position", 0).withSize(1,1).withPosition(2,3).getEntry();
        sbRightPosition = dtTab.add("Right Position", 0).withSize(1,1).withPosition(3,3).getEntry();
        
    }

    public void periodic() {
        //Constantly update the board
        
        double leftSpeed = mDriveBaseSubsystem.getLeftSpeed();
        double rightSpeed = mDriveBaseSubsystem.getRightSpeed();
        double gyroAngle = mDriveBaseSubsystem.getGyroAngle();
        double velocityConversionFactor = mDriveBaseSubsystem.getVelocityConversionFactor();
        double positions[] = mDriveBaseSubsystem.getPositions();

        sbLeftWheelSpeed.setDouble(leftSpeed);
        sbRightWheelSpeed.setDouble(rightSpeed);
        sbGyroAngle.setDouble(gyroAngle);
        sbVelocityConversionFactor.setDouble(velocityConversionFactor);
        sbLeftPosition.setDouble(positions[0]);
        sbRightPosition.setDouble(positions[1]);
        
    }
}
