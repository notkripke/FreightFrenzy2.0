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
public class redAutoDuck extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        Pose2d startPose = new Pose2d(-34, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        //****************************TRAJECTORIES************************************************

        Trajectory traj = drive.trajectoryBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-16.5, -47.5, Math.toRadians(165))) //-20, -43, 150
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(-30, -61.5, Math.toRadians(40)), Math.toRadians(180))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(-59.5, -64.5))
                .build();

        //************************VISION PROCESSING**************************************************************



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
                raiseLift(600, .9);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(600);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 590);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                robot.duck.setPower(0.4);
                sleep(150);
                robot.duck.setPower(.7);
                sleep(100);
                robot.duck.setPower(1);
                sleep(2000);
                robot.duck.setPower(0);
                Trajectory PARK = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                        .splineToLinearHeading(new Pose2d(-63.5, -41, Math.toRadians(0)), Math.toRadians(180))
                        .build();
                drive.followTrajectory(PARK);
                break;

            case 2:
                drive.followTrajectory(traj);
                raiseLift(1400, .9);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(600);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 1400);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                robot.duck.setPower(0.4);
                sleep(150);
                robot.duck.setPower(.7);
                sleep(100);
                robot.duck.setPower(1);
                sleep(2000);
                robot.duck.setPower(0);
                Trajectory PARK2 = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                        .splineToLinearHeading(new Pose2d(-63.5, -41, Math.toRadians(0)), Math.toRadians(180))
                        .build();
                drive.followTrajectory(PARK2);
                break;
            case 3:
                drive.followTrajectory(traj);
                raiseLift(2300, .9);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(600);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 2300);
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
                        .splineToLinearHeading(new Pose2d(-63.5, -41, Math.toRadians(0)), Math.toRadians(180))
                        .build();
                drive.followTrajectory(PARK3);
                break;

        }

    }

}