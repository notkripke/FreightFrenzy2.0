package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.CVPipeline;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;
//import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "drive")
public class blueAutoDuckTest extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override

    public void runOpMode() throws InterruptedException {


        initializeComponents();

        final long INITIAL_PAUSE = 0;
        final long SLEEP_TIME = 150;
        final int INIT_HEIGHT = robot.lift.getCurrentPosition();

        Pose2d startPose = new Pose2d(-34, 63.5, Math.toRadians(90));//-32, -22

        Trajectory dump1a = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-35, 30, Math.toRadians(90)))//dump 1 rough allignment
                .build();

        Trajectory dump1b = drive.trajectoryBuilder(dump1a.end())
                .lineToLinearHeading(new Pose2d(-32,28,Math.toRadians(110)))//dump 1 final allignment (low+mid)
                .build();

        Trajectory dump1c = drive.trajectoryBuilder(dump1a.end())
                .lineToLinearHeading(new Pose2d(-36.5,25,Math.toRadians(110)))//dump 1 final allignment (high)
                .build();

        Trajectory duck_approach1 = drive.trajectoryBuilder(dump1b.end())
                .splineToLinearHeading(new Pose2d(-57.5, 61, Math.toRadians(315)), Math.toRadians(170))//approach duck
                .build();

        Trajectory duck_approach2 = drive.trajectoryBuilder(duck_approach1.end())
                .splineToLinearHeading(new Pose2d(-58, 61, Math.toRadians(315)), Math.toRadians(170))//ensures duck contact
                .build();

        Trajectory find_duck = drive.trajectoryBuilder(duck_approach2.end())//Position for robot to detect duck location
                .splineToLinearHeading(new Pose2d(-18, 45, Math.toRadians(0)), Math.toRadians(0))
                .build();

        //------------------------------------------------------------------------------------------------------
        Trajectory duck_approach1c = drive.trajectoryBuilder(dump1c.end())
                .splineToLinearHeading(new Pose2d(-57.5, 61, Math.toRadians(315)), Math.toRadians(170))//approach duck
                .build();

        Trajectory duck_approach2c = drive.trajectoryBuilder(duck_approach1c.end())
                .splineToLinearHeading(new Pose2d(-58, 61, Math.toRadians(315)), Math.toRadians(170))//ensures duck contact
                .build();

        Trajectory find_duckc = drive.trajectoryBuilder(duck_approach2c.end())//Position for robot to detect duck location
                .splineToLinearHeading(new Pose2d(-50, 45, Math.toRadians(250)), Math.toRadians(275))
                .build();

        Trajectory dump2c = drive.trajectoryBuilder(find_duckc.end())//dump duck
                .splineToSplineHeading(new Pose2d(-36.5, 25, Math.toRadians(80)), Math.toRadians(260))
                .build();

        Trajectory parkc = drive.trajectoryBuilder(dump2c.end())//park in storage unti
                .lineToLinearHeading(new Pose2d(-63, 37, Math.toRadians(90)))
                .build();

        Trajectory dump2 = drive.trajectoryBuilder(find_duckc.end())//dump duck
                .splineToSplineHeading(new Pose2d(-36.5, 25, Math.toRadians(110)), Math.toRadians(0))
                .build();

        Trajectory park = drive.trajectoryBuilder(dump2.end())//park in storage unti
                .lineToLinearHeading(new Pose2d(-63, 37, Math.toRadians(90)))
                .build();

        drive.setPoseEstimate(startPose);


        startVisionProcessing();

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Position: ", Pipeline.getAnalysis());
            telemetry.addData("Avg1: ", Pipeline.getAvg1());
            telemetry.addData("Avg2: ", Pipeline.getAvg2());
            telemetry.addData("Avg3: ", Pipeline.getAvg3());
            if(Pipeline.getPos() == 1){
                LED("right");
            }
            if(Pipeline.getPos() == 2){
                LED("back");
            }
            if(Pipeline.getPos() == 3){
                LED("left");
            }
            telemetry.update();
        }

        //********************************************************************************************

        waitForStart();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });


        switch(Pipeline.getPos()){
            case 1:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(dump1a);
                sleep(SLEEP_TIME);
                drive.followTrajectory(dump1b);
                sleep(SLEEP_TIME);
                realAutoDump("low");
                drive.followTrajectory(duck_approach1);
                sleep(SLEEP_TIME);
                drive.followTrajectory(duck_approach2);
                sleep(SLEEP_TIME);
                robot.duck.setPower(-0.4);
                sleep(650);//550
                robot.duck.setPower(-0.4);
                sleep(1600);//1600
                robot.duck.setPower(0);
                robot.Intake2.setPower(-1);
                robot.Intake1.setPower(1);
                drive.followTrajectory(find_duck);
                sleep(500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                sleep(SLEEP_TIME);
                drive.followTrajectory(dump2);
                sleep(SLEEP_TIME);
                double LIFT_INIT = robot.lift.getCurrentPosition();
                double LIFT_POS = robot.lift.getCurrentPosition();
                double target = LIFT_MID;
                while(robot.lift.getCurrentPosition() < (target)){
                    robot.lift.setPower(1);
                }
                robot.lift.setPower(0);
                sleep(300);
                robot.outtake.setPosition(OUTTAKE_DOWN *1.1);
                sleep(700);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                double power = -1;
                double peak = robot.lift.getCurrentPosition() + LIFT_INIT;
                while((robot.lift.getCurrentPosition() - LIFT_INIT) > 50 /*&& !sensors.liftBot.getState()*/){


                    power = -(Math.abs(robot.lift.getCurrentPosition() / peak)) - 0.35;

                    robot.lift.setPower(power);

                    telemetry.addData("sensor: ", sensors.liftBot.getState());
                    telemetry.addData("pos: ", robot.lift.getCurrentPosition() - LIFT_INIT);
                    telemetry.addData("power: ", power);
                    telemetry.update();
                }
                robot.lift.setPower(0);
                drive.followTrajectory(park);
                break;

            case 2:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(dump1a);
                sleep(SLEEP_TIME);
                drive.followTrajectory(dump1c);
                sleep(SLEEP_TIME);
                LIFT_INIT = robot.lift.getCurrentPosition();
                LIFT_POS = robot.lift.getCurrentPosition();
                target = LIFT_MID;
                while(robot.lift.getCurrentPosition() < (target)){
                    robot.lift.setPower(1);
                }
                robot.lift.setPower(0);
                sleep(300);
                robot.outtake.setPosition(OUTTAKE_DOWN *1.1);
                sleep(700);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                power = -1;
                peak = robot.lift.getCurrentPosition() + LIFT_INIT;
                while((robot.lift.getCurrentPosition() - LIFT_INIT) > 20 /*&& !sensors.liftBot.getState()*/){


                    power = -(Math.abs(robot.lift.getCurrentPosition() / peak)) - 0.35;

                    robot.lift.setPower(power);

                    telemetry.addData("sensor: ", sensors.liftBot.getState());
                    telemetry.addData("pos: ", robot.lift.getCurrentPosition() - LIFT_INIT);
                    telemetry.addData("power: ", power);
                    telemetry.update();
                }
                robot.lift.setPower(0);
                drive.followTrajectory(duck_approach1c);
                sleep(SLEEP_TIME);
                drive.followTrajectory(duck_approach2c);
                sleep(SLEEP_TIME);
                robot.duck.setPower(-0.4);
                sleep(650);//550
                robot.duck.setPower(-0.4);
                sleep(1600);//1600
                robot.duck.setPower(0);
                robot.Intake2.setPower(-1);
                robot.Intake1.setPower(1);
                drive.followTrajectory(find_duckc);
                sleep(500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                sleep(SLEEP_TIME);
                drive.followTrajectory(dump2c);
                sleep(SLEEP_TIME);
                LIFT_INIT = robot.lift.getCurrentPosition();
                LIFT_POS = robot.lift.getCurrentPosition();
                target = LIFT_MID;
                while(robot.lift.getCurrentPosition() < (target)){
                    robot.lift.setPower(1);
                }
                robot.lift.setPower(0);
                sleep(300);
                robot.outtake.setPosition(OUTTAKE_DOWN *1.1);
                sleep(700);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                power = -1;
                peak = robot.lift.getCurrentPosition() + LIFT_INIT;
                while((robot.lift.getCurrentPosition() - LIFT_INIT) > 20 /*&& !sensors.liftBot.getState()*/){


                    power = -(Math.abs(robot.lift.getCurrentPosition() / peak)) - 0.35;

                    robot.lift.setPower(power);

                    telemetry.addData("sensor: ", sensors.liftBot.getState());
                    telemetry.addData("pos: ", robot.lift.getCurrentPosition() - LIFT_INIT);
                    telemetry.addData("power: ", power);
                    telemetry.update();
                }
                robot.lift.setPower(0);
                drive.followTrajectory(parkc);
                break;
            case 3:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(dump1a);
                sleep(SLEEP_TIME);
                drive.followTrajectory(dump1c);
                sleep(SLEEP_TIME);
                LIFT_INIT = robot.lift.getCurrentPosition();
                LIFT_POS = robot.lift.getCurrentPosition();
                target = LIFT_HIGH;
                while(robot.lift.getCurrentPosition() < (target)){
                    robot.lift.setPower(1);
                }
                robot.lift.setPower(0);
                sleep(300);
                robot.outtake.setPosition(OUTTAKE_DOWN *1.1);
                sleep(700);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                power = -1;
                peak = robot.lift.getCurrentPosition() + LIFT_INIT;
                while((robot.lift.getCurrentPosition() - LIFT_INIT) > 20 /*&& !sensors.liftBot.getState()*/){


                    power = -(Math.abs(robot.lift.getCurrentPosition() / peak)) - 0.35;

                    robot.lift.setPower(power);

                    telemetry.addData("sensor: ", sensors.liftBot.getState());
                    telemetry.addData("pos: ", robot.lift.getCurrentPosition() - LIFT_INIT);
                    telemetry.addData("power: ", power);
                    telemetry.update();
                }
                robot.lift.setPower(0);
                drive.followTrajectory(duck_approach1c);
                sleep(SLEEP_TIME);
                drive.followTrajectory(duck_approach2c);
                sleep(SLEEP_TIME);
                robot.duck.setPower(-0.4);
                sleep(650);//550
                robot.duck.setPower(-0.4);
                sleep(1600);//1600
                robot.duck.setPower(0);
                robot.Intake2.setPower(-1);
                robot.Intake1.setPower(1);
                drive.followTrajectory(find_duckc);
                sleep(500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                sleep(SLEEP_TIME);
                drive.followTrajectory(dump2c);
                sleep(SLEEP_TIME);
                LIFT_INIT = robot.lift.getCurrentPosition();
                LIFT_POS = robot.lift.getCurrentPosition();
                target = LIFT_MID;
                while(robot.lift.getCurrentPosition() < (target)){
                    robot.lift.setPower(1);
                }
                robot.lift.setPower(0);
                sleep(300);
                robot.outtake.setPosition(OUTTAKE_DOWN *1.1);
                sleep(700);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                power = -1;
                peak = robot.lift.getCurrentPosition() + LIFT_INIT;
                while((robot.lift.getCurrentPosition() - LIFT_INIT) > 20 /*&& !sensors.liftBot.getState()*/){


                    power = -(Math.abs(robot.lift.getCurrentPosition() / peak)) - 0.35;

                    robot.lift.setPower(power);

                    telemetry.addData("sensor: ", sensors.liftBot.getState());
                    telemetry.addData("pos: ", robot.lift.getCurrentPosition() - LIFT_INIT);
                    telemetry.addData("power: ", power);
                    telemetry.update();
                }
                robot.lift.setPower(0);
                drive.followTrajectory(parkc);
                break;

        }

    }



}