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

@TeleOp(group="main", name="duckTest")

@Config
public class duckTest extends GorillabotsCentral {

    @Override
    public void runOpMode() {

        initializeComponents();

        waitForStart();

        ElapsedTime duck_timer = new ElapsedTime();

        //boolean duck_trigger = false;
        String duck_trigger = "off";

        float duck_speed = 0;

        while (opModeIsActive()) {

           if(gamepad1.left_trigger >.4 && duck_trigger == "off"){
               duck_trigger = "red";
           }

           if(gamepad1.right_trigger > .4 && duck_trigger == "off"){
               duck_trigger = "blue";
           }

           switch(duck_trigger){
               case "off":
                   duck_speed = 0;
                   break;
               case "red":
                   duck_speed += .01;
                   sleep(9);
                   duck_timer.reset();
                   if(duck_speed >= 1){
                       duck_speed = 1;
                       if(duck_timer.time() >= 2.5){
                           duck_trigger = "off";
                       }
                   }
                   break;
               case "blue":
                   duck_speed -= .01;
                   sleep(9);
                   duck_timer.reset();
                   if(duck_speed <= -1){
                       duck_speed = -1;
                       if(duck_timer.time() >= 2.5){
                           duck_trigger = "off";
                       }
                   }
           }

           robot.duck.setPower(duck_speed);

        }
    }}
