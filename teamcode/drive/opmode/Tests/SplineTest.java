package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.opencv.core.Mat;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(-34, -61, Math.toRadians(270)));

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                //.splineToConstantHeading(new Vector2d(24, -24), Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(48, -24, Math.toRadians(90)), Math.toRadians(0))

                .lineToLinearHeading(new Pose2d(-16, -45, Math.toRadians(150)))
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

    }
}
