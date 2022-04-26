package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;


@TeleOp(group = "drive")
@Config
public class Teleop extends GorillabotsCentral { // 192.168.43.1:8080/dash

    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // telemetry uses
        // FTC app and Dashboard
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        ElapsedTime drive_timer = new ElapsedTime();

        boolean drive_check = false;

        String drive_state = "normal";

        String Lift_state = "stop";

        ElapsedTime duckPower = new ElapsedTime();

        String duck = "off";
        double r = 5.5;
        double u = 0.626;

        double k = Math.PI/lemniscate;

        double max = Math.sqrt(6.13/r);
        double time = 1000*Math.asin(1/max)/(k*max);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double LIFT_INIT = robot.lift.getCurrentPosition();

        double LIFT_POS;

        intake_to_dist_increment = 0;

        waitForStart();

        while (!isStopRequested()) {

            final double CEILING = LIFT_INIT + LIFT_CEILING;

            if(gamepad2.y){
                LIFT_INIT = robot.lift.getCurrentPosition();
            }

            LIFT_POS = robot.lift.getCurrentPosition();

            if(!gamepad1.left_bumper){
                intake_disabler = false; // BOOOM
            }

            if(gamepad1.left_bumper && intake_disabler == false){
                intakeToDist();
            }
            if(intake_disabler == true){
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
            }
            if(gamepad1.right_bumper){
                robot.Intake1.setPower(-1);
                robot.Intake2.setPower(1);
            }
            if(!gamepad1.left_bumper && !gamepad1.right_bumper){
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
            }

            if(gamepad1.left_trigger >.4 && gamepad1.right_trigger < .4){
                duck = "red";
            }
            if(gamepad1.right_trigger > .4 && gamepad1.left_trigger < .4){
                duck = "blue";
            }
            if(gamepad1.right_trigger < .4 && gamepad1.left_trigger < .4){
                duck = "off";
            }

            if(gamepad2.left_trigger >= .2 && gamepad2.right_trigger <= .2 && sensors.liftBot.getState()){//liftpos - ceiling < ceiling
                Lift_state = "down";
            }
            if(gamepad2.right_trigger >= .2 && gamepad2.left_trigger <= .2 && LIFT_POS < CEILING  /*sensors.checkSwitch() == false*/){
                Lift_state = "up";
            }
            if(gamepad2.left_trigger <= .2 && gamepad2.right_trigger <= .2 && Lift_state != "down"){
                Lift_state = "stop";
            }

            if(gamepad2.left_bumper){
                robot.outtake.setPosition(robot.outtake.getPosition() - 0.005);
            }
            if(gamepad2.right_bumper){
                robot.outtake.setPosition(robot.outtake.getPosition() + 0.005);
            }
            if(gamepad2.a){
                robot.outtake.setPosition(OUTTAKE_UP);
            }
            if(gamepad2.b){
                robot.outtake.setPosition(OUTTAKE_DOWN);
            }
            if(gamepad2.x){
                robot.outtake.setPosition(OUTTAKE_TILT);
            }

            switch (Lift_state){
                case "stop":
                    robot.lift.setPower(0);
                    break;
                case "down":
                    if(sensors.liftBot.getState()) {
                        robot.lift.setPower(-gamepad2.left_trigger);
                    }
                    if(!sensors.liftBot.getState()){
                        robot.lift.setPower(0);
                    }
                    break;
                case "up":
                    if(LIFT_POS >= CEILING) {
                        robot.lift.setPower(0);
                    }
                    if(LIFT_POS < LIFT_CEILING) {
                        robot.lift.setPower(gamepad2.right_trigger);
                    }
                    break;
            }

            switch(duck){
                case "off":
                    robot.duck.setPower(0);
                    duckPower.reset();
                    break;
                case "red":
                    if(duckPower.milliseconds() < time/1.5) {
                        robot.duck.setPower(0.60*max*Math.sin(1.5*max*k*duckPower.milliseconds()/1000));
                    }
                    if(duckPower.milliseconds() >= time/1.5){
                        robot.duck.setPower(0.60);
                    }
                    break;
                case "blue":
                    if(duckPower.milliseconds() < time/1.5) {
                        robot.duck.setPower(-0.60*max*Math.sin(1.5*max*k*duckPower.milliseconds()/1000));
                    }
                    if(duckPower.milliseconds() >= time/1.5){
                        robot.duck.setPower(-0.60);
                    }
                    break;
            }


            telemetry.addData("Outtake Pos: ", robot.outtake.getPosition());
            telemetry.addData("lift height: ", LIFT_POS);
            telemetry.addData("Freight? ", freightCheck());
            telemetry.addData("Distance from init: ", LIFT_POS - LIFT_INIT);
            telemetry.addData("Lift state: ", Lift_state);
            telemetry.addData("Ceiling", CEILING);
            telemetry.addData("look here: ",intake_disabler);
            telemetry.update();

            if(gamepad1.a && drive_timer.time() >= 0.4){
                drive_timer.reset();
                drive_check =! drive_check;
            }

            if(drive_check){
                drive_state = "gap";
            }
            if(!drive_check){
                drive_state = "normal";
            }

            switch(drive_state){
                case "normal":
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                    break;
                case "gap":
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x + 0.35,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
            }

            telemetry.addData("Drive state: ", drive_state);
            telemetry.update();

        }

        }
    }
    


