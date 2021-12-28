package org.firstinspires.ftc.teamcode.drive.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class RobotHardware
{
    Telemetry tele;

    public DcMotor lift, Intake1, Intake2, duck;
    public Servo outtake;

    public static int LIFT_CEILING = 3000;
    public static double LIFT_SPEED_MULTIPLIER = .5;
    public static double OUTTAKE_UP = 0;
    public static double OUTTAKE_DOWN = .6;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry)
    {
        tele = telemetry;



        lift = hardwareMap.dcMotor.get("lift");
        Intake1 = hardwareMap.dcMotor.get("intake1");
        Intake2 = hardwareMap.dcMotor.get("intake2");
        duck = hardwareMap.dcMotor.get("duck");
        outtake = hardwareMap.servo.get("outtake");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}
