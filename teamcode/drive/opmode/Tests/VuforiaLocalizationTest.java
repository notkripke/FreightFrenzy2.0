package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.opmode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;

/*
This is an opMode designed to test vuforia's localization capabilities. The robot will drive around, and the
localization may be started by holding B. The robot will take this information and attempt to drive to the
desired location (determined by Waypoint) autonomously after pressing BOTH A & Y
*/


@TeleOp(group = "drive")
public class VuforiaLocalizationTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        boolean vision = false;

        final double Scan_Time = 3.00;

        final Pose2d Waypoint = new Pose2d(0,0,Math.toRadians(0));

        waitForStart();

        while (!isStopRequested()) {

            if(gamepad1.b) {
                vision = true;
            }
            if(!gamepad1.b){
                vision = false;
            }

            if(vision){

                drive.setWeightedDrivePower(new Pose2d(-0, 0,0));
                drive.update();
                sleep(150);
                VuforiaScan(Scan_Time, vision);
                drive.setPoseEstimate(VuforiaLocalizedPose2d());
            }


            if(!vision){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            }

            if(gamepad1.a && gamepad1.y){
                Trajectory MoveToWaypoint = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading(Waypoint).build();
                drive.followTrajectory(MoveToWaypoint);
            }
            telemetry.addData("VuforiaPose2d (X,Y,R): ", VuforiaLocalizedPose2d());
            telemetry.update();


        }


        }
    }
