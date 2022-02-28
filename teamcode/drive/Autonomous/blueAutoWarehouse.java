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
public class blueAutoWarehouse extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        final long INITIAL_PAUSE = 0;
        final long SLEEP_TIME = 400;

        Pose2d startPose = new Pose2d(12, 63.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                //.back(14)
                .lineToLinearHeading(new Pose2d(12, 49.5, Math.toRadians(90)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(12, 49.5, Math.toRadians(0)))
                .back(20)
                .build();
        Trajectory traj3a = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-12, 49, Math.toRadians(0)))
                .build();

        Trajectory park1a = drive.trajectoryBuilder(traj3a.end())
                .strafeLeft(16)
                .build();

        Trajectory park2a = drive.trajectoryBuilder(park1a.end())
                //.lineToConstantHeading(new Vector2d(35.6, -78)))
                .lineToLinearHeading(new Pose2d(36, 72.5, Math.toRadians(0)))
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-10, 53.2, Math.toRadians(0)))
                .build();

        Trajectory park1b = drive.trajectoryBuilder(traj3b.end())
                .strafeLeft(13.5)
                .build();

        Trajectory park2b = drive.trajectoryBuilder(park1b.end())
                //.lineToConstantHeading(new Vector2d(38, -83))
                .lineToLinearHeading(new Pose2d(36, 75.5, Math.toRadians(10)))
                .build();
        Trajectory park3 = drive.trajectoryBuilder(new Pose2d(36, 67.5, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(39, -45.5, Math.toRadians(180)))
                .strafeRight(14)
                .build();
        Trajectory park4 = drive.trajectoryBuilder(new Pose2d(38, 44.5, Math.toRadians(0)))
                //.strafeLeft(36)
                .forward(6)
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
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(1000);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(690);
                robot.lift.setPower(0);
                sleep(200);
                drive.followTrajectory(park1a);
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0.5,
                                0.4,
                                0
                        )
                );
                sleep(1800);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                sleep(SLEEP_TIME);
                creepIntake("forwards", 4500);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                robot.Intake1.setPower(-0.75);
                robot.Intake2.setPower(-0.75);
                sleep(1500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                drive.setWeightedDrivePower(new Pose2d(0, -0.45, 0));
                sleep(1400);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
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
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(1000);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(920);
                robot.lift.setPower(0);
                sleep(200);
                drive.followTrajectory(park1a);
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0.5,
                                0.4,
                                0
                        )
                );
                sleep(1800);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                sleep(SLEEP_TIME);
                creepIntake("forwards", 4500);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                robot.Intake1.setPower(-0.75);
                robot.Intake2.setPower(-0.75);
                sleep(1500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                drive.setWeightedDrivePower(new Pose2d(0, -0.45, 0));
                sleep(1400);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
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
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN*0.95);
                sleep(1400);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(1485);
                robot.lift.setPower(0);
                sleep(200);
                drive.followTrajectory(park1b);
                sleep(SLEEP_TIME);
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0.5,
                                0.4,
                                0
                        )
                );
                sleep(1800);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                sleep(SLEEP_TIME);
                creepIntake("forwards", 4500);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                robot.Intake1.setPower(-0.75);
                robot.Intake2.setPower(-0.75);
                sleep(1500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                drive.setWeightedDrivePower(new Pose2d(0, -0.45, 0));
                sleep(1400);
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                //drive.followTrajectory(park3);
                //drive.followTrajectory(park4);
                break;

        }

    }

}