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

@Autonomous(group = "drive")
public class SplineTest extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Pose2d localization_variable = new Pose2d();

        drive.setPoseEstimate(startPose);
        

        //test 1 (driivng through barrier test)
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(20, 5))
                .build();
        //






        drive.followTrajectory(traj);

        //drive.setWeightedDrivePower(new Pose2d(.5,.5,0));
        //localization_variable = drive.getCurrentPosition();
        //Trajectory traj1 = drive.trajectoryBuilder(localization_variable)
        //.lineToConstantHeading(new Vector2d(10, 10))
        //.build();



    }
}
