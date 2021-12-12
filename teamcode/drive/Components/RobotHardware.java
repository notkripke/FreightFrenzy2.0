package org.firstinspires.ftc.teamcode.drive.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware
{
    Telemetry tele;

    public DcMotor lift, intake, spinner;
    public Servo swivel;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry)
    {
        tele = telemetry;

        lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");
        spinner = hardwareMap.dcMotor.get("spinner");
        swivel = hardwareMap.servo.get("swivel");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}
