package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class redAutoDuckTest extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        final long INITIAL_PAUSE = 0;
        final long SLEEP_TIME = 150;
        final int INIT_HEIGHT = robot.lift.getCurrentPosition();

        Pose2d startPose = new Pose2d(-34, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);


        Trajectory duck1 = drive.trajectoryBuilder(startPose)
                .back(4.5)
                .build();
        Trajectory duck2 = drive.trajectoryBuilder(duck1.end())
                .lineToLinearHeading(new Pose2d(-57.5, -60, Math.toRadians(45)))
                .build();
        Trajectory duck3 = drive.trajectoryBuilder(duck2.end())
                .lineToConstantHeading(new Vector2d(-57.5, -61))
                .build();
        Trajectory duck4 = drive.trajectoryBuilder(duck3.end())
                .lineToLinearHeading(new Pose2d(-52.5, -51, Math.toRadians(180)))
                .build();

        Trajectory hub1 = drive.trajectoryBuilder(duck4.end())
                .lineToLinearHeading(new Pose2d(-12, -55.5, Math.toRadians(180)))
                .build();
        Trajectory hub2a = drive.trajectoryBuilder(hub1.end())
                .lineToConstantHeading(new Vector2d(-12, -48))
                .build();
        Trajectory park1a = drive.trajectoryBuilder(new Pose2d(-12, -48, Math.toRadians(180)))
                .strafeLeft(8.5)
                .build();
        Trajectory hub2b = drive.trajectoryBuilder(hub1.end())
                .lineToConstantHeading(new Vector2d(-12, -48))
                .build();
        Trajectory park1b = drive.trajectoryBuilder(new Pose2d(-12, -48, Math.toRadians(180)))
                .strafeLeft(16.5)
                .build();
        Trajectory hub2c = drive.trajectoryBuilder(hub1.end())
                .lineToLinearHeading(new Pose2d(-10, -55.5, Math.toRadians(0)))
                .build();
        Trajectory park1c = drive.trajectoryBuilder(new Pose2d(-12, -55.5, Math.toRadians(180)))
                .strafeLeft(1)
                .build();
        Trajectory park2 = drive.trajectoryBuilder(park1c.end())
                .lineToLinearHeading(new Pose2d(-63, -36, Math.toRadians(90)))
                //.splineToSplineHeading(new Pose2d(-63, -31, Math.toRadians(90)), Math.toRadians(90))
                .build();

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


        switch(Pipeline.getPos()){
            case 1:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(duck1);
                sleep(SLEEP_TIME);
                drive.followTrajectory(duck2);
                sleep(SLEEP_TIME);
                drive.followTrajectory(duck3);
                sleep(SLEEP_TIME);
                robot.duck.setPower(0.4);
                sleep(650);//550
                robot.duck.setPower(0.4);
                sleep(1600);//1600
                robot.duck.setPower(0);
                drive.followTrajectory(duck4);
                sleep(SLEEP_TIME);
                drive.followTrajectory(hub1);
                sleep(SLEEP_TIME);
                drive.followTrajectory(hub2a);
                sleep(SLEEP_TIME);
                sleep(SLEEP_TIME);
                /*raiseLiftTeleop(INIT_HEIGHT);
                sleep(600);
                robot.lift.setPower(-0.65);
                sleep(1600);
                robot.lift.setPower(0);*/
                robot.lift.setPower(.8);
                sleep(700);
                robot.lift.setPower(0);
                sleep(400);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(1000);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(650);
                robot.lift.setPower(0);
                sleep(200);
                drive.followTrajectory(park1a);
                sleep(400);
                drive.followTrajectory(park2);
                break;

            case 2:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(duck1);
                sleep(SLEEP_TIME);
                drive.followTrajectory(duck2);
                sleep(SLEEP_TIME);
                drive.followTrajectory(duck3);
                sleep(SLEEP_TIME);
                robot.duck.setPower(0.4);
                sleep(650);//550
                robot.duck.setPower(0.4);
                sleep(1600);//1600
                robot.duck.setPower(0);
                drive.followTrajectory(duck4);
                sleep(SLEEP_TIME);
                drive.followTrajectory(hub1);
                sleep(SLEEP_TIME);
                drive.followTrajectory(hub2a);
                sleep(SLEEP_TIME);
                drive.turn(Math.toRadians(-190));
                sleep(SLEEP_TIME);
                /*raiseLiftTeleop(INIT_HEIGHT);
                sleep(600);
                robot.lift.setPower(-0.65);
                sleep(1600);
                robot.lift.setPower(0);*/
                robot.lift.setPower(.8);
                sleep(950);
                robot.lift.setPower(0);
                sleep(400);
                robot.outtake.setPosition(OUTTAKE_DOWN*1.1);
                sleep(1000);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(900);
                robot.lift.setPower(0);
                sleep(200);
                drive.followTrajectory(park1a);
                sleep(400);
                drive.followTrajectory(park2);
                break;
            case 3:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(duck1);
                sleep(SLEEP_TIME);
                drive.followTrajectory(duck2);
                sleep(SLEEP_TIME);
                drive.followTrajectory(duck3);
                sleep(SLEEP_TIME);
                robot.duck.setPower(0.4);
                sleep(650);//550
                robot.duck.setPower(0.4);
                sleep(1600);//1600
                robot.duck.setPower(0);
                drive.followTrajectory(duck4);
                sleep(SLEEP_TIME);
                drive.followTrajectory(hub1);
                sleep(SLEEP_TIME);
                drive.followTrajectory(hub2c);
                sleep(SLEEP_TIME);
                drive.turn(Math.toRadians(-190));
                sleep(SLEEP_TIME);
                /*raiseLiftTeleop(INIT_HEIGHT);
                sleep(600);
                robot.lift.setPower(-0.65);
                sleep(1600);
                robot.lift.setPower(0);*/
                robot.lift.setPower(.8);
                sleep(1540);//1495
                robot.lift.setPower(0);
                sleep(400);
                robot.outtake.setPosition(OUTTAKE_DOWN*1.1);
                sleep(1000);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(1490);
                robot.lift.setPower(0);
                sleep(200);
                drive.followTrajectory(park1a);
                sleep(400);
                drive.followTrajectory(park2);
                break;

        }

    }

}