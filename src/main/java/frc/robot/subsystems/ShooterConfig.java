// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.subsystems;

/** Add your docs here. */
public class ShooterConfig {
    //ShooterConfig [] DistanceArray;
    double Distance;
    double Angle;
    double RPM;
    

    public ShooterConfig(double DDistance, double DAngle, double DRPM ){
        this.Distance = DDistance;
        this.Angle = DAngle;
        this.RPM = DRPM;
    }

    public double getDistance(){
        return this.Distance;
    }
    
    public double getAngleDegrees(){
        return this.Angle;
    }
    public double getVelocity(){
        return this.RPM;
    }
    public double[] getVelocityAndAngle(){
        double[] returnArray = new double[2];
        returnArray[0] = getVelocity();
        returnArray[1] = getAngleDegrees();
        return returnArray;
    }

    public static double[] Interprolate(ShooterConfig obj1, ShooterConfig obj2,double distance){
        double [] returnArray = new double [2];
        //Obj1 > Obj2
        //returnArray[0] = (obj1.getVelocity()*(obj1.getDistance()-distance) + obj2.getVelocity()*(distance-obj2.getDistance()) )/(obj1.getDistance()-obj2.getDistance());;
        //returnArray[1] = ( obj2.getAngleDegrees()*(obj1.getDistance()-distance) + obj2.getAngleDegrees()*(distance-obj2.getDistance()) )/(obj1.getDistance()-obj2.getDistance());
        returnArray[0] = ((obj1.getVelocity()-obj2.getVelocity())/(obj1.getDistance()-obj2.getDistance()))*(distance-obj2.getDistance())+obj2.getDistance();
        returnArray[1] = ((obj1.getAngleDegrees()-obj2.getAngleDegrees())/(obj1.getDistance()-obj2.getDistance()))*(distance-obj2.getDistance())+obj2.getDistance();
        return returnArray;
 
    }
}
