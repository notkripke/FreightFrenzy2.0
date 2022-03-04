package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.opmode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;
import org.opencv.core.Mat;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class SplineTest extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);


        Trajectory traj = drive.trajectoryBuilder(startPose)

                .back(30)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .strafeRight(24)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(0, 30, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(6, 30), Math.toRadians(180))
                .build();



        drive.followTrajectory(traj);
        sleep(500);
        drive.followTrajectory(traj2);
        sleep(500);
        drive.followTrajectory(traj3);

    }
}
