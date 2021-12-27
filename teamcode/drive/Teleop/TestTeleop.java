package org.firstinspires.ftc.teamcode.drive.Teleop;

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
import org.firstinspires.ftc.teamcode.drive.Components.Sensors;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@Config
@TeleOp(group = "drive")
public class TestTeleop extends LinearOpMode {

    Sensors sensors = new Sensors(hardwareMap, telemetry);

    public static double OUTTAKE_UP = 0;
    public static double OUTTAKE_DOWN = .6;

    static double LIFT_SPEED_MULTIPLIER = .5;


    static double LIFT_CEILING = 3000;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        DcMotor Intake1, Intake2, duck, lift;
        Servo outtake;
        outtake = hardwareMap.servo.get("outtake");
        final double SPINNER_SPEED = .08;
        lift = hardwareMap.dcMotor.get("lift");
        Intake1 = hardwareMap.dcMotor.get("intake1");
        Intake2 = hardwareMap.dcMotor.get("intake2");
        duck = hardwareMap.dcMotor.get("duck");
        duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        String Lift_state = "stop";

        double LIFT_POS;

        waitForStart();

        while (!isStopRequested()) {

            LIFT_POS = lift.getCurrentPosition();

            if(gamepad1.left_bumper){
                Intake1.setPower(.9);
            }
            if(gamepad1.right_bumper){
                Intake2.setPower(-.9);
            }
            if(!gamepad1.left_bumper){
                Intake1.setPower(0);
            }
            if(!gamepad1.right_bumper){
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
            if(gamepad2.right_trigger >= .2 && gamepad2.left_trigger <= .5 && -LIFT_POS - LIFT_CEILING > LIFT_CEILING){
                Lift_state = "up";
            }

            if(gamepad2.left_trigger >= .2 && gamepad2.right_trigger <= .2){
                Lift_state = "down";
            }

            if((gamepad2.left_trigger <= .2 && gamepad2.right_trigger <= .2) || -LIFT_POS - LIFT_CEILING <= LIFT_CEILING && Lift_state != "down"){
                Lift_state = "stop";
            }

            if(gamepad2.left_bumper){
                outtake.setPosition(outtake.getPosition() - 0.01);
            }
            if(gamepad2.right_bumper){
                outtake.setPosition(outtake.getPosition() + 0.01);
            }

            switch (Lift_state){
                case "stop":
                    lift.setPower(0);
                    break;
                case "down":
                    lift.setPower(-gamepad2.left_trigger * LIFT_SPEED_MULTIPLIER);
                    break;
                case "up":
                    lift.setPower(gamepad2.right_trigger * LIFT_SPEED_MULTIPLIER);
            }

            telemetry.addData("Outtake Pos: ", outtake.getPosition());
            telemetry.addData("lift height: ", lift.getCurrentPosition());
            telemetry.addData("Lift ceiling: ", LIFT_CEILING);
            telemetry.addData("Dist. 'till ceiling: ", Math.abs(LIFT_CEILING - LIFT_POS));
            telemetry.addData("Distance sensor: ", sensors.getDistanceDist());
            telemetry.addData("Touch sensor: ", sensors.checkTouch());
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


