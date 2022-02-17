package frc.robot.subsystems.Tabs;

import frc.robot.subsystems.DriveBaseSubsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TabDriveBase {

    // each box on the shuffleboard, sb stands for shuffleboard
    private NetworkTableEntry sbLeftWheelSpeed;
    private NetworkTableEntry sbRightWheelSpeed;
    private NetworkTableEntry sbGyroAngle;
    private NetworkTableEntry sbLeftPosition;
    private NetworkTableEntry sbRightPosition;

    // TODO: other indicators for motor controllers: if inverted, what motors they're following, + other factors important to driver and debugging

    // DriveBaseSubsystem object
    private DriveBaseSubsystem mDriveBaseSubsystem;

    private AHRS mGyro;

    public TabDriveBase(DriveBaseSubsystem d) {
        this.mDriveBaseSubsystem = d;
        this.mGyro = d.getGyro();

        // gets the DriveBase tab, if it doesn't exist, create it with the name "DriveBase"
        ShuffleboardTab dtTab = Shuffleboard.getTab("DriveBase");

        sbLeftWheelSpeed = dtTab.add("Left Wheel Speed", 0).withSize(2, 2).withPosition(0, 0).getEntry();
        sbRightWheelSpeed = dtTab.add("Right Wheel Speed", 0).withSize(2, 2).withPosition(6, 0).getEntry();
        sbGyroAngle = dtTab.add("Gyro Angle", 0).withSize(2, 2).withPosition(3, 0).getEntry();
        Shuffleboard.getTab("DriveBase").add(mGyro);    // adds a gyro compass indicator

        sbLeftPosition = dtTab.add("Left Position", 0).withSize(2,2).withPosition(0,3).getEntry();
        sbRightPosition = dtTab.add("Right Position", 0).withSize(2,2).withPosition(6,3).getEntry();
        
    }

    public void periodic() {
        //Constantly update the board
        if(mDriveBaseSubsystem != null) {
            double leftSpeed = mDriveBaseSubsystem.getLeftSpeed();
            double rightSpeed = mDriveBaseSubsystem.getRightSpeed();
            double gyroAngle = mGyro.getAngle();
            double positions[] = mDriveBaseSubsystem.getPositions();

            sbLeftWheelSpeed.setDouble(leftSpeed);
            sbRightWheelSpeed.setDouble(rightSpeed);
            sbGyroAngle.setDouble(gyroAngle);
            sbLeftPosition.setDouble(positions[0]);
            sbRightPosition.setDouble(positions[1]);

            // TODO: compare encoder value to value from biconsumer, should be equal
            SmartDashboard.putNumber("left speed (rpm) [encoder]", leftSpeed);
            SmartDashboard.putNumber("right speed (rpm) [encoder]", rightSpeed);
        }
        
    }
}
