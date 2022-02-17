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
public class redAutoDuck extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        final long INITIAL_PAUSE = 0;
        final long SLEEP_TIME = 400;
        final int INIT_HEIGHT = robot.lift.getCurrentPosition();

        Pose2d startPose = new Pose2d(-34, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);


        Trajectory duck1 = drive.trajectoryBuilder(startPose)
                .back(4.5)
                .build();
        Trajectory duck2 = drive.trajectoryBuilder(duck1.end())
                .lineToLinearHeading(new Pose2d(-57.5, -60, Math.toRadians(45)))
                .build();

        Trajectory hub1 = drive.trajectoryBuilder(duck2.end())
                .lineToLinearHeading(new Pose2d(-12, -55.5, Math.toRadians(0)))
                .build();
        Trajectory hub2 = drive.trajectoryBuilder(hub1.end())
                .lineToConstantHeading(new Vector2d(-12, -50))
                .build();
        Trajectory park1 = drive.trajectoryBuilder(new Pose2d(-12, -50, Math.toRadians(180)))
                .strafeLeft(16.5)
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
                sleep(SLEEP_TIME);
                robot.duck.setPower(0.4);
                sleep(500);
                robot.duck.setPower(0.75);
                sleep(1000);
                robot.duck.setPower(0);
                drive.followTrajectory(hub1);
                sleep(SLEEP_TIME);
                drive.followTrajectory(hub2);
                sleep(SLEEP_TIME);
                drive.turn(Math.toRadians(-180));
                sleep(SLEEP_TIME+1000);
                /*raiseLiftTeleop(INIT_HEIGHT);
                sleep(600);
                robot.lift.setPower(-0.65);
                sleep(1600);
                robot.lift.setPower(0);*/
                drive.followTrajectory(park1);
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -0.5,
                                0.35,
                                0
                        )
                );
                sleep(900);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                creepIntake("back", 3000);
                break;

            case 2:

                break;
            case 3:

                break;

        }

    }

}