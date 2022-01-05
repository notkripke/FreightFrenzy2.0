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

        Pose2d startPose = new Pose2d(-34, -61, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-16.5, -46, Math.toRadians(165))) //-20, -43, 150
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(-30, -60, Math.toRadians(40)), Math.toRadians(180))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(-61.5, -63))
                .build();


        robot.outtake.setPosition(OUTTAKE_UP);
        drive.followTrajectory(traj);
        sleep(1000);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        robot.duck.setPower(.4);
        sleep(300);
        robot.duck.setPower(.75);
        sleep(300);
        robot.duck.setPower(1);
        sleep(1500);
        robot.duck.setPower(0);


        sleep(2000);

    }
}
