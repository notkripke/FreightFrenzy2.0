package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.Sensors;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;

@TeleOp(group="main", name="duckTest")
@Disabled
@Config
public class duckTest extends GorillabotsCentral {

    @Override
    public void runOpMode() {

        initializeComponents();

        ElapsedTime duckPower = new ElapsedTime();

        double angVel = 0;
        double angAcc = 0;

        double maxVelCaro = 435*0.10472*0.2135;
        double r = 5.5;
        double u = 0.626;

        double k = Math.PI/lemniscate;

        double max = 0.8*Math.sqrt(6.13/r);
        double time = 1000*Math.asin(1/max)/(k*max);

        String duck = "off";
        String mode = "sine";

        double sinConst = Math.sqrt(9.8*u/r);

        double accConst = 1500;

        waitForStart();

        while (opModeIsActive()) {
            if(mode == "measure") {//
                boolean stopCheck = false;

                if (gamepad1.left_trigger > .4 && gamepad1.right_trigger < .4 && !stopCheck) {
                    robot.duck.setPower(duckPower.milliseconds() / accConst);

                    angVel = maxVelCaro * duckPower.milliseconds() / accConst;
                    angAcc = maxVelCaro * (1000 / accConst);

                    if(gamepad1.x){
                        stopCheck = false;
                    }
                }

                telemetry.addData("Angular Velocity: ", angVel);
                telemetry.addData("Angular Acceleration: ", angAcc);
                telemetry.addData("Time: ", duckPower.milliseconds());
                telemetry.update();
            }

            if(mode == "lemntest") {
                if (gamepad1.right_trigger > .4 && gamepad1.left_trigger < .4) {
                    double speed = sinlemn(sinConst * duckPower.milliseconds());
                    if(speed < 0.95) {
                        robot.duck.setPower(speed);
                    }
                    if(speed >= 0.95){
                        robot.duck.setPower(1);
                    }

                    telemetry.addData("Angular Velocity: ", sinConst * sinlemn(sinConst * duckPower.milliseconds()));
                    telemetry.addData("Radius: ", r);
                    telemetry.addData("Static Coefficient: ", u);
                    telemetry.addData("Maximum Velocity: ", sinConst);
                    telemetry.addData("Time: ", duckPower.milliseconds());
                    telemetry.update();
                }
                if(gamepad1.right_trigger < .4 && gamepad1.left_trigger < .4) {
                    robot.duck.setPower(0);

                    telemetry.addData("Radius: ", r);
                    telemetry.addData("Static Coefficient: ", u);
                    telemetry.addData("Maximum Velocity: ", sinConst);
                    telemetry.addData("Time: ", duckPower.milliseconds());
                }
            }
            if(mode == "sine"){
                if(gamepad1.left_trigger >.4 && gamepad1.right_trigger < .4){
                    duck = "red";
                }
                if(gamepad1.right_trigger > .4 && gamepad1.left_trigger < .4){
                    duck = "blue";
                }
                if(gamepad1.right_trigger < .4 && gamepad1.left_trigger < .4){
                    duck = "off";
                }

                switch(duck){
                    case "off":
                        robot.duck.setPower(0);
                        duckPower.reset();
                        break;
                    case "red":
                        if(duckPower.milliseconds() < time) {
                            robot.duck.setPower(max*Math.sin(max*k*duckPower.milliseconds()/1000));
                        }
                        if(duckPower.milliseconds() >= time){
                            robot.duck.setPower(1);
                        }
                        break;
                    case "blue":
                        if(duckPower.milliseconds() < time) {
                            robot.duck.setPower(-max*Math.sin(max*k*duckPower.milliseconds()/1000));
                        }
                        if(duckPower.milliseconds() >= time){
                            robot.duck.setPower(-1);
                        }
                        break;
                }
            }

            /*
            if(gamepad1.a) {
                mode = "test";
            }
            if(gamepad1.b){
                mode = "measure";
            }
            */

           drive.setWeightedDrivePower(
                   new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
        }

    }
}
