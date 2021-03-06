package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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


@Autonomous(group = "drive")
public class RedAutoWarehouseCycle extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        final long INITIAL_PAUSE = 0;
        final long SLEEP_TIME = 150;

        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory initialOffset = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, -59, Math.toRadians(270)))
                .build();

        Trajectory hubApproach = drive.trajectoryBuilder(initialOffset.end())
                .splineToLinearHeading(new Pose2d(-3, -49, Math.toRadians(207)), Math.toRadians(90))
                .build();

        Trajectory hubAdjust = drive.trajectoryBuilder(hubApproach.end())
                .lineToLinearHeading(new Pose2d(-3, -46, Math.toRadians(207)))
                .build();

        Trajectory hubApproachTop = drive.trajectoryBuilder(initialOffset.end())
                .splineToLinearHeading(new Pose2d(-3, -52, Math.toRadians(200)), Math.toRadians(90))
                .build();

        Trajectory hubAdjustTop = drive.trajectoryBuilder(hubApproachTop.end())
                .lineToLinearHeading(new Pose2d(-3, -51, Math.toRadians(202)))
                .build();

        Trajectory warehouse1 = drive.trajectoryBuilder(hubAdjust.end())
                .splineToLinearHeading(new Pose2d(0, -65.5, Math.toRadians(180)), Math.toRadians(270))
                .build();

        Trajectory warehouse2 = drive.trajectoryBuilder(warehouse1.end())
                .lineToConstantHeading(new Vector2d(35, -65.5))
                .build();

        Trajectory warehouse1Top = drive.trajectoryBuilder(hubAdjustTop.end())
                .splineToLinearHeading(new Pose2d(0, -65.5, Math.toRadians(180)), Math.toRadians(270))
                .build();

        Trajectory warehouse2Top = drive.trajectoryBuilder(warehouse1Top.end())
                .lineToConstantHeading(new Vector2d(35, -65.5))
                .build();

        Trajectory secondHubApproach = drive.trajectoryBuilder(new Pose2d(0, -65.5, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(0, -44, Math.toRadians(215)), Math.toRadians(90))
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
                drive.followTrajectory(hubApproach);
                sleep(200);
                drive.followTrajectory(hubAdjust);
                sleep(200);
                realAutoDump("bottom");
                sleep(200);
                drive.followTrajectory(warehouse1);
                sleep(SLEEP_TIME);
                drive.followTrajectory(warehouse2);
                sleep(SLEEP_TIME);
                creepIntakeOffset("backwards", 10000);
                robot.Intake2.setPower(1);
                robot.Intake1.setPower(1);

                Trajectory warehouseExit1 = drive.trajectoryBuilder(offsetPose)
                        .lineToLinearHeading(new Pose2d(0, -65.5, Math.toRadians(180)))
                        .build();

                sleep(300);

                drive.followTrajectory(warehouseExit1);
                sleep(200);
                robot.Intake1.setPower(0);
                robot.Intake2.setPower(0);
                drive.followTrajectory(secondHubApproach);
                sleep(200);
                robot.Intake2.setPower(-1);
                robot.Intake1.setPower(1);
                sleep(500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                realAutoDump("middle");
                drive.followTrajectory(warehouse1);
                sleep(200);
                drive.followTrajectory(warehouse2);
                sleep(200);
                creepIntake("backwards", 4000);
                break;

            case 2:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(hubApproach);
                sleep(200);
                drive.followTrajectory(hubAdjust);
                sleep(200);
                realAutoDump("middle");
                sleep(200);
                drive.followTrajectory(warehouse1);
                sleep(SLEEP_TIME);
                drive.followTrajectory(warehouse2);
                sleep(SLEEP_TIME);
                creepIntakeOffset("backwards", 10000);
                robot.Intake2.setPower(1);
                robot.Intake1.setPower(1);

                Trajectory warehouseExit2 = drive.trajectoryBuilder(offsetPose)
                        .lineToLinearHeading(new Pose2d(0, -65.5, Math.toRadians(180)))
                        .build();

                sleep(300);

                drive.followTrajectory(warehouseExit2);
                sleep(200);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                drive.followTrajectory(secondHubApproach);
                sleep(200);
                robot.Intake2.setPower(-1);
                robot.Intake1.setPower(1);
                sleep(500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                realAutoDump("middle");
                sleep(200);
                drive.followTrajectory(warehouse1);
                sleep(200);
                drive.followTrajectory(warehouse2);
                sleep(200);
                creepIntake("backwards", 4000);
                break;
            case 3:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(hubApproachTop);
                sleep(200);
                drive.followTrajectory(hubAdjustTop);
                sleep(200);
                realAutoDump("high");
                sleep(200);
                drive.followTrajectory(warehouse1Top);
                sleep(SLEEP_TIME);
                drive.followTrajectory(warehouse2Top);
                sleep(SLEEP_TIME);
                creepIntakeOffset("backwards", 10000);
                robot.Intake2.setPower(1);
                robot.Intake1.setPower(1);

                Trajectory warehouseExit3 = drive.trajectoryBuilder(offsetPose)
                        .lineToLinearHeading(new Pose2d(0, -65.5, Math.toRadians(180)))
                        .build();

                sleep(300);

                drive.followTrajectory(warehouseExit3);
                sleep(200);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                drive.followTrajectory(secondHubApproach);
                sleep(200);
                robot.Intake2.setPower(-1);
                robot.Intake1.setPower(1);
                sleep(500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                realAutoDump("middle");
                sleep(200);
                drive.followTrajectory(warehouse1);
                sleep(200);
                drive.followTrajectory(warehouse2);
                sleep(200);
                creepIntake("backwards", 4000);
                break;

        }

    }

}