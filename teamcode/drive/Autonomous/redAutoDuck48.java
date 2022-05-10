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
//import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(group = "drive")
public class redAutoDuck48 extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        final long INITIAL_PAUSE = 0;
        final long SLEEP_TIME = 150;
        final int INIT_HEIGHT = robot.lift.getCurrentPosition();

        Pose2d startPose = new Pose2d(-34, -63.5, Math.toRadians(270));//-32, -22

        Trajectory dump1a = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-35, -30, Math.toRadians(90)))//dump 1 rough allignment
                .build();

        Trajectory dump1b = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-31,-25,Math.toRadians(80)))//dump 1 final allignment (low+mid)
                .build();

        Trajectory dump1c = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-36.5,-25,Math.toRadians(80)))//dump 1 final allignment (high)
                .build();

        Trajectory duck_approach1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(-57.5, -61, Math.toRadians(45)), Math.toRadians(190))//approach duck
                .build();

        Trajectory duck_approach2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(-57.5, -61, Math.toRadians(45)), Math.toRadians(190))//ensures duck contact
                .build();

        Trajectory find_duck = drive.trajectoryBuilder(drive.getPoseEstimate())//Position for robot to detect duck location
                .splineToLinearHeading(new Pose2d(-56, -50, Math.toRadians(70)), Math.toRadians(70))
                .build();

        /****************  DUCK INTAKES. NUMBERED 1-4 FROM LEFT TO RIGHT RELATIVE TO CAMERA   *************/

        Trajectory duck_intake1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-57, -64, Math.toRadians(90)))
                .build();

        Trajectory dump2 = drive.trajectoryBuilder(drive.getPoseEstimate())//dump duck
                .splineToSplineHeading(new Pose2d(-36.5, -25, Math.toRadians(80)), Math.toRadians(0))
                .build();

        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())//park in storage unti
                .lineToLinearHeading(new Pose2d(-63, -37, Math.toRadians(90)))
                .build();

        drive.setPoseEstimate(startPose);


        startVisionProcessing();

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Position: ", Pipeline.getAnalysis());
            telemetry.addData("Avg1: ", Pipeline.getAvg1());
            telemetry.addData("Avg2: ", Pipeline.getAvg2());
            telemetry.addData("Avg3: ", Pipeline.getAvg3());
            if(Pipeline.getPos() == 1){
                LED("right");
            }
            if(Pipeline.getPos() == 2){
                LED("back");
            }
            if(Pipeline.getPos() == 3){
                LED("left");
            }
            telemetry.update();
        }

        //********************************************************************************************

        waitForStart();


        switch(Pipeline.getPos()){
            case 1:
                sleep(INITIAL_PAUSE);
                drive.followTrajectory(dump1a);
                drive.followTrajectory(dump1b);
                drive.followTrajectory(duck_approach1);
                drive.followTrajectory(duck_approach2);
                robot.Intake1.setPower(1);
                robot.Intake2.setPower(-1);
                drive.followTrajectory(duck_intake1);
                sleep(500);
                robot.Intake2.setPower(0);
                robot.Intake1.setPower(0);
                drive.followTrajectory(dump2);
                drive.followTrajectory(park);
                break;

            case 2:
                sleep(INITIAL_PAUSE);

                break;
            case 3:
                sleep(INITIAL_PAUSE);

                break;

        }

    }

}