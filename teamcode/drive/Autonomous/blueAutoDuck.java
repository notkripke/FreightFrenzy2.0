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
public class blueAutoDuck extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(-34, 61, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        //****************************TRAJECTORIES************************************************

        Trajectory TO_HUB = drive.trajectoryBuilder(startPose, true)
                .lineToLinearHeading(new Pose2d(-20, -50, Math.toRadians(150)))
                .build();

        Trajectory TO_DUCK = drive.trajectoryBuilder(TO_HUB.end(), false)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(355)), Math.toRadians(180))
                .build();

        Trajectory DUCK_BACKUP = drive.trajectoryBuilder(TO_DUCK.end(), false)
                .lineToConstantHeading(new Vector2d(-55, -55))
                .build();

        Trajectory PARK = drive.trajectoryBuilder(DUCK_BACKUP.end(), false)
                .splineToLinearHeading(new Pose2d(-60, -35.5, Math.toRadians(0)), Math.toRadians(180))
                .build();
        //************************VISION PROCESSING**************************************************************

        CVPipeline Pipeline;
        OpenCvCamera webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        Pipeline = new CVPipeline();
        webcam.setPipeline(Pipeline);
        webcam.openCameraDevice();
        webcam.setPipeline(Pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

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
                drive.followTrajectory(TO_HUB);
                raiseLift(600, .9);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(600);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 590);
                drive.followTrajectory(TO_DUCK);
                drive.followTrajectory(DUCK_BACKUP);
                robot.duck.setPower(0.4);
                sleep(150);
                robot.duck.setPower(.7);
                sleep(100);
                robot.duck.setPower(1);
                sleep(2000);
                robot.duck.setPower(0);
                drive.followTrajectory(PARK);
                break;

            case 2:
                drive.followTrajectory(TO_HUB);
                raiseLift(1400, .9);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(600);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 1400);
                drive.followTrajectory(TO_DUCK);
                drive.followTrajectory(DUCK_BACKUP);
                robot.duck.setPower(0.4);
                sleep(150);
                robot.duck.setPower(.7);
                sleep(100);
                robot.duck.setPower(1);
                sleep(2000);
                robot.duck.setPower(0);
                drive.followTrajectory(PARK);
                break;
            case 3:
                drive.followTrajectory(TO_HUB);
                raiseLift(2300, .9);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(600);
                robot.outtake.setPosition(OUTTAKE_UP);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                robot.outtake.setPosition(OUTTAKE_UP);
                lowerLift(.7, 2300);
                drive.followTrajectory(TO_DUCK);
                drive.followTrajectory(DUCK_BACKUP);
                robot.duck.setPower(0.4);
                sleep(150);
                robot.duck.setPower(.7);
                sleep(100);
                robot.duck.setPower(1);
                sleep(2000);
                robot.duck.setPower(0);
                drive.followTrajectory(PARK);
                break;

        }

    }

}