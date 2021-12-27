package org.firstinspires.ftc.teamcode.drive.Components;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors
{

    //public Rev2mDistanceSensor rangeB;
    //public Rev2mDistanceSensor rangeL;
    //public Rev2mDistanceSensor rangeFL;
    //public Rev2mDistanceSensor rangeF;
    //public Rev2mDistanceSensor rangeR;
    public Rev2mDistanceSensor dist;
    public TouchSensor touch;

    public Sensors(HardwareMap hardwareMap, Telemetry telemetry)
    {

        //rangeL = hardwareMap.get(Rev2mDistanceSensor.class, "rangeL");
        //rangeF = hardwareMap.get(Rev2mDistanceSensor.class, "rangeF");
        //rangeFL = hardwareMap.get(Rev2mDistanceSensor.class, "rangeFL");
        //rangeB = hardwareMap.get(Rev2mDistanceSensor.class, "rangeB");
        //rangeR = hardwareMap.get(Rev2mDistanceSensor.class, "rangeR");
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
        touch = hardwareMap.touchSensor.get("touch");
    }

    public double getDistanceDist(){
        return dist.getDistance(DistanceUnit.INCH);
    }

    public double checkTouch(){
        return touch.getValue();
    }

     //public double getDistanceB(){
       //return rangeB.getDistance(DistanceUnit.INCH);
    //}

   /* public double getDistanceL(){
        return rangeL.getDistance(DistanceUnit.INCH);
    }
    public double getDistanceFL(){
        return rangeFL.getDistance(DistanceUnit.INCH);
    }
    public double getDistanceF(){
        return rangeF.getDistance(DistanceUnit.INCH);
    }*/




    //public double getDistanceR(){
      //  return rangeR.getDistance(DistanceUnit.INCH);
    //}
}
