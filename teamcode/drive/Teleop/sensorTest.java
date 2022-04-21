package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.Sensors;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
@TeleOp(group="main", name="sensorTest")
@Disabled
@Config
public class sensorTest extends GorillabotsCentral {



    @Override
    public void runOpMode() {

        initializeComponents();

        waitForStart();

        while (opModeIsActive()) {

            //telemetry.addData("distance: ", sensors.checkTouch());
            //telemetry.addData("Limit switch is pressed?: ", sensors.checkSwitch());
            if(freightCheck() == "LOADED"){
                LED("all");
            }
            if(freightCheck() != "LOADED"){
                LED("none");
            }
            if (sensors.liftBot.getState()){
                telemetry.addData("touch", "is pressed");
            }
            if (!sensors.liftBot.getState()){
                telemetry.addData("touch", "is not pressed");
            }
            telemetry.addData("touch", sensors.liftBot.getState());
            telemetry.update();
        }
    }}
