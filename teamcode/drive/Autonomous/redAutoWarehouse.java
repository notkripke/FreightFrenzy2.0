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
public class redAutoWarehouse extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        final long INITIAL_PAUSE = 0;
        final long SLEEP_TIME = 400;

        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .back(14)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(12, -49.5, Math.toRadians(180)))
                .forward(20)
                .build();
        Trajectory traj3a = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-12, -49, Math.toRadians(180)))
                .build();

        Trajectory park1a = drive.trajectoryBuilder(traj3a.end())
                .strafeLeft(16)
                .build();

        Trajectory park2a = drive.trajectoryBuilder(park1a.end())
                .lineToConstantHeading(new Vector2d(38, -78))
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-12, -49, Math.toRadians(180)))
                .build();

        Trajectory park1b = drive.trajectoryBuilder(traj3b.end())
                .strafeLeft(16)
                .build();

        Trajectory park2b = drive.trajectoryBuilder(park1b.end())
                .lineToConstantHeading(new Vector2d(38, -78))
                .build();
        Trajectory park3 = drive.trajectoryBuilder(new Pose2d(38, -67.5, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(38, -38, Math.toRadians(270)))
                .build();
        Trajectory park4 = drive.trajectoryBuilder(park3.end())
                .strafeLeft(36)
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
                sleep(SLEEP_TIME);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(traj2);
                sleep(SLEEP_TIME);
                drive.followTrajectory(traj3a);
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
                sleep(SLEEP_TIME);
                drive.followTrajectory(park2a);
                sleep(SLEEP_TIME);
                drive.followTrajectory(park3);
                sleep(SLEEP_TIME);
                drive.followTrajectory(park4);
                creepIntake("forwards", 6000);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                sleep(SLEEP_TIME);
                robot.Intake1.setPower(0.75);
                robot.Intake2.setPower(0.75);
                sleep(6000);
                break;

            case 2:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(traj);
                sleep(SLEEP_TIME);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(traj2);
                sleep(SLEEP_TIME);
                drive.followTrajectory(traj3a);
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
                sleep(SLEEP_TIME);
                drive.followTrajectory(park2a);
                sleep(SLEEP_TIME);
                drive.followTrajectory(park3);
                sleep(SLEEP_TIME);
                drive.followTrajectory(park4);
                creepIntake("forwards", 6000);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                sleep(SLEEP_TIME);
                robot.Intake1.setPower(0.75);
                robot.Intake2.setPower(0.75);
                sleep(6000);
                break;
            case 3:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(traj);
                sleep(SLEEP_TIME);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(traj2);
                sleep(SLEEP_TIME);
                drive.followTrajectory(traj3b);
                robot.lift.setPower(.8);
                sleep(1495);
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
                drive.followTrajectory(park1b);
                sleep(SLEEP_TIME);
                drive.followTrajectory(park2b);
                sleep(SLEEP_TIME);
                drive.followTrajectory(park3);
                sleep(SLEEP_TIME);
                drive.followTrajectory(park4);
                creepIntake("forwards", 6000);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                sleep(SLEEP_TIME);
                robot.Intake1.setPower(0.75);
                robot.Intake2.setPower(0.75);
                sleep(6000);
                break;

        }

    }

}