package org.firstinspires.ftc.teamcode.drive.Components;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors
{
   // DigitalChannel LimitSwitch;
    public Rev2mDistanceSensor dist;
    //public RevTouchSensor touch;

    public Sensors(HardwareMap hardwareMap, Telemetry telemetry)
    {
        //LimitSwitch = hardwareMap.get(DigitalChannel.class, "switch");
        //LimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
    }

    public double getDistanceDist(){
        return dist.getDistance(DistanceUnit.INCH);
    }

    /*public boolean checkSwitch(){
        //return LimitSwitch.getState();
        boolean isPressed;
        if(LimitSwitch.getState() == true){
            isPressed = false;
        }
        else{
            isPressed = true;
        }
        return isPressed;
    }*/
}
