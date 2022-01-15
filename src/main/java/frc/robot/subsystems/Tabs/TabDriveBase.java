package frc.robot.subsystems.Tabs;

import frc.robot.subsystems.DriveBaseSubsystem;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class TabDriveBase {

    // each box on the shuffleboard, sb stands for shuffleboard
    private NetworkTableEntry sbLeftWheelSpeed;
    private NetworkTableEntry sbRightWheelSpeed;
    // TODO: other indicators for motor controllers: if inverted, what motors they're following, + other factors important to driver and debugging

    // DriveBaseSubsystem object
    private DriveBaseSubsystem mDriveBaseSubsystem;

    public TabDriveBase(DriveBaseSubsystem d) {
        this.mDriveBaseSubsystem = d;

        // gets the DriveBase tab, if it doesn't exist, create it with the name "DriveBase"
        ShuffleboardTab dtTab = Shuffleboard.getTab("DriveBase");

        sbLeftWheelSpeed = dtTab.add("Left Wheel Speed", 0).withSize(2, 2).withPosition(1, 2).getEntry();
        sbRightWheelSpeed = dtTab.add("Right Wheel Speed", 0).withSize(2, 2).withPosition(3, 2).getEntry();

        
    }

    public void periodic() {
        // contantly update the board
        
        // TODO: call back to DriveBaseSubsystem to send back information, implement getter methods
        double leftSpeed = mDriveBaseSubsystem.getLeftSpeed();
        double rightSpeed = mDriveBaseSubsystem.getRightSpeed();

        sbLeftWheelSpeed.setDouble(leftSpeed);
        sbRightWheelSpeed.setDouble(leftSpeed);
        
    }
}
