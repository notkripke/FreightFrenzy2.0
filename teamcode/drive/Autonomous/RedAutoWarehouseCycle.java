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
public class RedAutoWarehouseCycle extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        final long INITIAL_PAUSE = 0;
        final long SLEEP_TIME = 150;

        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                //.back(14)
                .lineToLinearHeading(new Pose2d(12, -49.5, Math.toRadians(270)))
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
                //.lineToConstantHeading(new Vector2d(35.6, -78)))
                .lineToLinearHeading(new Pose2d(36, -72.5, Math.toRadians(180)))
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-12, -53.2, Math.toRadians(180)))
                .build();

        Trajectory park1b = drive.trajectoryBuilder(traj3b.end())
                .strafeLeft(16)
                .build();

        Trajectory park2b = drive.trajectoryBuilder(park1b.end())
                //.lineToConstantHeading(new Vector2d(38, -83))
                .lineToLinearHeading(new Pose2d(36, -71.5, Math.toRadians(180)))
                .build();

        Trajectory secondDump1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(0, -71.5, Math.toRadians(180)))
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
                sleep(500);
                creepIntake("forwards", 1000);
                sleep(500);
                Trajectory test = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d())
                        .build();
                sleep(500);
                drive.followTrajectory(test);
               /*
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
                creepIntake("backwards", 6000);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                robot.Intake1.setPower(0.75);
                robot.Intake2.setPower(0.75);
                sleep(1000);
                 */
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
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(1000);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(920);
                robot.lift.setPower(0);
                sleep(200);
                drive.followTrajectory(park1a);
                sleep(SLEEP_TIME);
                drive.followTrajectory(park2a);
                sleep(SLEEP_TIME);
                creepIntake("backwards", 6000);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                robot.Intake1.setPower(0.75);
                robot.Intake2.setPower(0.75);
                sleep(1000);
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
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(1000);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(1375);
                robot.lift.setPower(0);
                sleep(200);
                drive.followTrajectory(park1b);
                sleep(SLEEP_TIME);
                drive.followTrajectory(park2b);
                sleep(SLEEP_TIME);
                creepIntake("backwards", 6000);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                robot.Intake1.setPower(0.75);
                robot.Intake2.setPower(0.75);
                sleep(1000);
                break;

        }

    }

}