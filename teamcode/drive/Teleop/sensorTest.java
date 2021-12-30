package org.firstinspires.ftc.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Components.Sensors;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;

@TeleOp(group="main", name="sensorTest")

@Config
public class sensorTest extends LinearOpMode {



    @Override
    public void runOpMode() {

        Sensors sensors;
        sensors = new Sensors(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            //telemetry.addData("distance: ", sensors.checkTouch());
            telemetry.addData("Touch: ", sensors.getDistanceDist()); //6 threshold
            telemetry.update();
        }
    }}
