package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.Sensors;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;

@Config
@TeleOp(group = "drive")
public class AutomatedTeleop extends GorillabotsCentral {         // 192.168.43.1:8080/dash

    Sensors sensors = new Sensors(hardwareMap, telemetry);

    public static double OUTTAKE_UP = 0;
    public static double OUTTAKE_DOWN = .6;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        DcMotor Intake1, Intake2, duck, lift;
        Servo outtake;
        outtake = hardwareMap.servo.get("outtake");
        final double SPINNER_SPEED = .08;
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake1 = hardwareMap.dcMotor.get("intake1");
        Intake2 = hardwareMap.dcMotor.get("intake2");
        duck = hardwareMap.dcMotor.get("duck");
        duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        String Lift_state = "stop";

        int LIFT_POS;

        final int LIFT_INIT = lift.getCurrentPosition();

        waitForStart();

        while (!isStopRequested()) {

            LIFT_POS = lift.getCurrentPosition();

            if(gamepad1.left_bumper && sensors.getDistanceDist() >= 5){ //Intakes; NOT toggles, spin when held.
                Intake1.setPower(.9);
            }
            if(gamepad1.right_bumper && sensors.getDistanceDist() >= 5){
                Intake2.setPower(-.9);
            }
            if(!gamepad1.left_bumper || sensors.getDistanceDist() <= 5){
                Intake1.setPower(0);
            }
            if(!gamepad1.right_bumper || sensors.getDistanceDist() <= 5){
                Intake2.setPower(0);
            }


            if(gamepad1.left_trigger >= 0.15){
                duck.setPower(-gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger >= 0.15){
                duck.setPower(gamepad1.right_trigger);
            }
            if(gamepad1.left_trigger < .15 && gamepad1.right_trigger < .15){
                duck.setPower(0);
            }


            if(gamepad2.b && Lift_state != "top"){
                Lift_state = "top";
            }
            if(gamepad2.a && Lift_state != "top" && Lift_state != "shared"){
                Lift_state = "shared";
            }
            if(gamepad2.x && Lift_state != "bottom"){
                Lift_state = "bottom";
            }

            switch (Lift_state){
                case "top":
                    if(LIFT_POS <= LIFT_CEILING && LIFT_POS >= LIFT_CEILING - 150){
                     lift.setPower(.02);
                     outtake.setPosition(OUTTAKE_DOWN);
                    }
                    if(LIFT_POS < LIFT_CEILING - 150) {
                        raiseLift(LIFT_CEILING - 100, LIFT_SPEED);
                        outtake.setPosition(OUTTAKE_UP);
                    }
                    break;
                case "shared":
                    if(LIFT_POS <= SHARED_HEIGHT && LIFT_POS >= SHARED_HEIGHT - 100){
                        lift.setPower(0.02);
                        outtake.setPosition(OUTTAKE_DOWN);
                    }
                    if(LIFT_POS < SHARED_HEIGHT - 100) {
                        raiseLift(SHARED_HEIGHT, LIFT_SPEED);
                        outtake.setPosition(OUTTAKE_UP);
                    }
                    break;
                case "bottom":
                    outtake.setPosition(OUTTAKE_UP);
                    lowerLift(LIFT_SPEED * .9, LIFT_POS - LIFT_INIT);
            }

            telemetry.addData("Outtake Pos: ", outtake.getPosition());
            telemetry.addData("lift height: ", lift.getCurrentPosition());
            telemetry.addData("Lift ceiling: ", LIFT_CEILING);
            telemetry.addData("Dist. 'till ceiling: ", Math.abs(LIFT_CEILING - LIFT_POS));
            telemetry.addData("Distance sensor: ", sensors.getDistanceDist());
            telemetry.update();

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


