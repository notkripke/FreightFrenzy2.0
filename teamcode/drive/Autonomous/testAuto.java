package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;
//import org.opencv.core.Mat;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class testAuto extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToConstantHeading(new Vector2d(5, 10))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj2);

        creepIntakeOffset("forwards", 7000);

        Trajectory traj = drive.trajectoryBuilder(offsetPose)
                .lineToLinearHeading(new Pose2d(-5, 15, Math.toRadians(90)))
                .build();

        drive.followTrajectory(traj);
    }
}
