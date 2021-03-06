package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors
{
   // DigitalChannel LimitSwitch;

    public Rev2mDistanceSensor dist;
    public DigitalChannel liftBot;
    public Rev2mDistanceSensor sidedist;
    public DigitalChannel flLED;
    public DigitalChannel frLED;
    public DigitalChannel blLED;
    public DigitalChannel brLED;


    public Sensors(HardwareMap hardwareMap, Telemetry telemetry)
    {

        liftBot = hardwareMap.get(DigitalChannel.class,"liftBot");
        liftBot.setMode(DigitalChannel.Mode.INPUT);
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
        sidedist = hardwareMap.get(Rev2mDistanceSensor.class, "sidedist");
        frLED = hardwareMap.get(DigitalChannel.class, "frLED");
        flLED = hardwareMap.get(DigitalChannel.class, "flLED");
        blLED = hardwareMap.get(DigitalChannel.class, "blLED");
        brLED = hardwareMap.get(DigitalChannel.class, "brLED");
    }

    public double getDistanceDist(){
        return dist.getDistance(DistanceUnit.INCH);
    }
    public double getDistanceSideDist() {return sidedist.getDistance(DistanceUnit.INCH);}
}
