package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Autonomous(group = "drive")
public class UredAutoWarehouse extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        final long INITIAL_PAUSE = 0;

        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-4, -45, Math.toRadians(210)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(11, -67.5, Math.toRadians(180)), Math.toRadians(0))
                .build();
        Trajectory park1 = drive.trajectoryBuilder(traj2.end())
                .back(35)
                .build();
        Trajectory park2 = drive.trajectoryBuilder(park1.end())
                .strafeLeft(5)
                .build();
        Trajectory intake = drive.trajectoryBuilder(park2.end())
                .lineToLinearHeading(new Pose2d(67, -62, Math.toRadians(270)))
                .build();
        Trajectory finalpark = drive.trajectoryBuilder(intake.end())
                .lineToLinearHeading(new Pose2d(67, 62, Math.toRadians(270)))
                .strafeLeft(16)
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
                raiseLift(1140, .9);
                sleep(500);
                robot.outtake.setPosition(OUTTAKE_UP * 0.35);
                sleep(2000);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 1315);
                drive.followTrajectory(traj2);
                drive.turn(Math.toRadians(-10));
                drive.followTrajectory(park1);
                drive.followTrajectory(park2);
                drive.followTrajectory(intake);
                creepIntake("forwards", 6000);
                robot.Intake1.setPower(-1);
                robot.Intake2.setPower(1);
                drive.followTrajectory(finalpark);
                sleep(500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                break;

            case 2:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(traj);
                raiseLift(1800, .9);
                sleep(500);
                robot.outtake.setPosition(OUTTAKE_UP * .2);
                sleep(1200);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 1799);
                drive.followTrajectory(traj2);
                drive.turn(Math.toRadians(-10));
                drive.followTrajectory(park1);
                drive.followTrajectory(park2);
                drive.followTrajectory(intake);
                creepIntake("forwards", 6000);
                robot.Intake1.setPower(-1);
                robot.Intake2.setPower(1);
                drive.followTrajectory(finalpark);
                sleep(500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                break;
            case 3:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(traj);
                raiseLift(2350, -.7);//raise
                sleep(2000);
                robot.outtake.setPosition(OUTTAKE_UP*0.1);
                sleep(1200);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 2300);
                drive.followTrajectory(traj2);
                drive.turn(Math.toRadians(-10));
                drive.followTrajectory(park1);
                drive.followTrajectory(park2);
                drive.followTrajectory(intake);
                creepIntake("forwards", 6000);
                robot.Intake1.setPower(-1);
                robot.Intake2.setPower(1);
                drive.followTrajectory(finalpark);
                sleep(500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                break;

        }

    }

}