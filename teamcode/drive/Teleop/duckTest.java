package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.Sensors;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;

@TeleOp(group="main", name="duckTest")

@Config
public class duckTest extends GorillabotsCentral {

    @Override
    public void runOpMode() {

        initializeComponents();

        ElapsedTime duckPower = new ElapsedTime();

        String duck_trigger = "off";

        waitForStart();

        while (opModeIsActive()) {

           if(gamepad1.left_trigger >.4 && gamepad1.right_trigger < .4){
               duck_trigger = "red";
           }
           if(gamepad1.right_trigger > .4 && gamepad1.left_trigger > .4){
               duck_trigger = "blue";
           }
           if(gamepad1.right_trigger < .4 && gamepad1.left_trigger < .4){
               duck_trigger = "off";
           }

           switch(duck_trigger){
               case "off":
                   robot.duck.setPower(0);
                   duckPower.reset();
                   break;
               case "red":
                   if(duckPower.milliseconds() < 775) {
                       robot.duck.setPower(duckPower.milliseconds()/800);
                   }
                   if(duckPower.milliseconds() >= 775){
                       robot.duck.setPower(1);
                   }
                   break;
               case "blue":
                   if(duckPower.milliseconds() < 775) {
                       robot.duck.setPower(-duckPower.milliseconds()/800);
                   }
                   if(duckPower.milliseconds() >= 775){
                       robot.duck.setPower(-1);
                   }
                   break;
           }
        }
    }}
