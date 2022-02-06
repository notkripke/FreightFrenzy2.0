package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

        double angVel = 0;
        double angAcc = 0;

        double r = 0.15;
        double u = 0.626;

        String mode = "measure";

        double sinConst = Math.sqrt(9.8*u/r);

        double accConst = 1500;

        waitForStart();

        while (opModeIsActive()) {
            if(mode == "measure") {
                if (gamepad1.left_trigger > .4 && gamepad1.right_trigger < .4) {
                    robot.duck.setPower(duckPower.milliseconds() / accConst);

                    angVel = 6.97559232 * duckPower.milliseconds() / accConst;
                    angAcc = 6.97559232 * (1000 / accConst);

                    telemetry.addData("Angular Velocity: ", angVel);
                    telemetry.addData("Angular Acceleration: ", angAcc);
                    telemetry.addData("Time: ", duckPower.milliseconds());
                    telemetry.update();
                }
                if (gamepad1.right_trigger < .4 && gamepad1.left_trigger < .4) {
                    if (duckPower.milliseconds() != 0) {
                        angVel = 6.97559232 * duckPower.milliseconds() / accConst;
                        angAcc = 6.97559232 * (1000 / accConst);
                    }

                    robot.duck.setPower(0);
                    duckPower.reset();

                    telemetry.addData("Angular Velocity: ", angVel);
                    telemetry.addData("Angular Acceleration: ", angAcc);
                    telemetry.addData("Time: ", duckPower.milliseconds());
                    telemetry.update();
                }
            }

            if(mode == "test") {
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

            if(gamepad1.a) {
                mode = "test";
            }
            if(gamepad1.b){
                mode = "measure";
            }

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
