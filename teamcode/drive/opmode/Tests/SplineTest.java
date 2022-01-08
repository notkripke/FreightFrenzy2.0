package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.opmode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;
import org.opencv.core.Mat;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(-34, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-16.5, -46.5, Math.toRadians(165))) //raise y
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(-30, -62.5, Math.toRadians(40)), Math.toRadians(180))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(-58.5, -66.5))
                .build();



        drive.followTrajectory(traj);
        raiseLift(2350, -.7);//raise
        sleep(2000);
        robot.outtake.setPosition(OUTTAKE_DOWN);
        sleep(600);
        robot.outtake.setPosition(OUTTAKE_UP);
        robot.outtake.setPosition(OUTTAKE_DOWN);
        robot.outtake.setPosition(OUTTAKE_UP);
        lowerLift(.7, 2200);
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

    }
}
