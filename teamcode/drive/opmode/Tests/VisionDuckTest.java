package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.opmode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.DuckPipeline;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;
//import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(group = "drive")
public class VisionDuckTest extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        final long INITIAL_PAUSE = 0;
        final long SLEEP_TIME = 150;
        final int INIT_HEIGHT = robot.lift.getCurrentPosition();

        Pose2d startPose = new Pose2d(-34, 63.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);


        Trajectory leftOut = drive.trajectoryBuilder(startPose)
                .back(4.5)
                .build();
        Trajectory leftIn = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-62, 55, Math.toRadians(315)))
                .build();
        Trajectory rightIn = drive.trajectoryBuilder(startPose)
                .back(4.5)
                .build();
        Trajectory rightOut = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-62, 55, Math.toRadians(315)))
                .build();

        startVisionProcessing();

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Position: ", PipelineD.getAnalysis());
            telemetry.addData("Avg1: ", PipelineD.getAvg1());
            telemetry.addData("Avg2: ", PipelineD.getAvg2());
            telemetry.addData("Avg3: ", PipelineD.getAvg3());
            telemetry.addData("Avg4: ", PipelineD.getAvg4());
            if(PipelineD.getPos() == 1){
                LED("right");
            }
            if(PipelineD.getPos() == 2){
                LED("back");
            }
            if(PipelineD.getPos() == 3){
                LED("left");
            }
            if(PipelineD.getPos() == 4){
                LED("front");
            }
            telemetry.update();
        }

        //********************************************************************************************

        waitForStart();


        switch(PipelineD.getPos()){
            case 1:

            case 2:

            case 3:

            case 4:

        }

    }

}