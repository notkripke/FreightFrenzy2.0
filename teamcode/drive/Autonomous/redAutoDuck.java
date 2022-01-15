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

        Pose2d startPose = new Pose2d(-34, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-16.5, -47.5, Math.toRadians(165))) //raise y
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(-30, -62.5, Math.toRadians(40)), Math.toRadians(180))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(-58.5, -66.5))
                .build();


        startVisionProcessing();

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Position: ", Pipeline.getAnalysis());
            telemetry.addData("Avg1: ", Pipeline.getAvg1());
            telemetry.addData("Avg2: ", Pipeline.getAvg2());
            telemetry.addData("Avg3: ", Pipeline.getAvg3());
            telemetry.update();
        }

        //********************************************************************************************

        waitForStart();


        switch(Pipeline.getPos()){
            case 1:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(traj);
                raiseLift(1350, .9);
                sleep(500);
                robot.outtake.setPosition(OUTTAKE_UP * 0.2);
                sleep(2000);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 1300);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                robot.duck.setPower(0.4);
                sleep(150);
                robot.duck.setPower(.7);
                sleep(100);
                robot.duck.setPower(1);
                sleep(2000);
                robot.duck.setPower(0);
                Trajectory PARK = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                        .splineToLinearHeading(new Pose2d(-64.5, -41, Math.toRadians(0)), Math.toRadians(180))
                        .build();
                drive.followTrajectory(PARK);
                break;

            case 2:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(traj);
                raiseLift(1900, .9);
                sleep(500);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(1200);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 1899);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                robot.duck.setPower(0.4);
                sleep(150);
                robot.duck.setPower(.7);
                sleep(100);
                robot.duck.setPower(1);
                sleep(2000);
                robot.duck.setPower(0);
                Trajectory PARK2 = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                        .splineToLinearHeading(new Pose2d(-64.5, -41, Math.toRadians(0)), Math.toRadians(180))
                        .build();
                drive.followTrajectory(PARK2);
                break;
            case 3:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(traj);
                raiseLift(2350, .9);//raise
                sleep(500);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(600);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 2300);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                robot.duck.setPower(0.4);
                sleep(150);
                robot.duck.setPower(.7);
                sleep(100);
                robot.duck.setPower(1);
                sleep(2000);
                robot.duck.setPower(0);
                Trajectory PARK3 = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                        .splineToLinearHeading(new Pose2d(-64.2, -41, Math.toRadians(0)), Math.toRadians(180))
                        .build();
                drive.followTrajectory(PARK3);
                break;

        }

    }

}