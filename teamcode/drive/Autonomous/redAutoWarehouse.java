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
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(group = "drive")
public class redAutoWarehouse extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-4, -45.5, Math.toRadians(210)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(11, -68, Math.toRadians(180)), Math.toRadians(0))
                .build();
        Trajectory park1 = drive.trajectoryBuilder(traj2.end())
                .back(24)
                .build();
        Trajectory park2 = drive.trajectoryBuilder(park1.end())
                .strafeRight(22)
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
                drive.followTrajectory(traj);
                raiseLift(1350, .9);
                robot.outtake.setPosition(OUTTAKE_UP * 0.2);
                sleep(2000);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 1300);
                drive.followTrajectory(traj2);
                drive.followTrajectory(park1);
                drive.followTrajectory(park2);

                break;

            case 2:
                drive.followTrajectory(traj);
                raiseLift(1800, .9);
                robot.outtake.setPosition(OUTTAKE_UP * .2);
                sleep(1200);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 1799);
                drive.followTrajectory(traj2);
                drive.followTrajectory(park1);
                drive.followTrajectory(park2);

                break;
            case 3:
                drive.followTrajectory(traj);
                raiseLift(2350, -.7);//raise
                sleep(2000);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(1200);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 2300);
                drive.followTrajectory(traj2);
                drive.followTrajectory(park1);
                drive.followTrajectory(park2);

                break;

        }

    }

}