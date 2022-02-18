package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.opmode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")//22
public class ApproachTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start_pos = new Pose2d(12,-58,Math.toRadians(90));
        Pose2d hub_node = new Pose2d(-5.5,-43, Math.toRadians(180));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(start_pos)
                .splineToLinearHeading(hub_node, Math.toRadians(90))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .lineToLinearHeading(new Pose2d(15, -61, Math.toRadians(0)))
                .back(20)
                .build();

        drive.followTrajectory(traj);
        sleep(3500);
        drive.followTrajectory(traj2);
    }
}
