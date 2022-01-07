package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Autonomous;

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
public class testAuto extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)

                .lineToLinearHeading(new Pose2d(2, -45, Math.toRadians(220)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(16, -66, Math.toRadians(0)), Math.toRadians(350))
                .build();
        Trajectory park1 = drive.trajectoryBuilder(traj2.end())
                .forward(24)
                .build();
        Trajectory park2 = drive.trajectoryBuilder(park1.end())
                .strafeLeft(22)
                .build();
        
        /* ALTERNATIVE TO THE ABOVE PARKING MEASURES:
        Trajectory park = drive.tragectoryBuilder(trag2.end())
                .splineToConstantHeading(new Vector2d(46, -44), Math.toRadians(100))
                .build();
         */




        drive.followTrajectory(traj);
        raiseLift(2300, -.7);//raise
        robot.outtake.setPosition(OUTTAKE_DOWN);
        sleep(600);
        robot.outtake.setPosition(OUTTAKE_UP);
        robot.outtake.setPosition(OUTTAKE_DOWN);
        robot.outtake.setPosition(OUTTAKE_UP);
        lowerLift(.7, 1800);
        drive.followTrajectory(traj2);
        drive.followTrajectory(park1);
        drive.followTrajectory(park2);


    }
}
