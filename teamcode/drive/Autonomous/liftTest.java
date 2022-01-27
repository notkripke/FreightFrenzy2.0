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

@Autonomous(group = "drive")
public class liftTest extends GorillabotsCentral {// 192.168.43.1:8080/dash
    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();

        int init_height = robot.lift.getCurrentPosition();

        while(!isStarted() && !isStopRequested()) {
            double s = 94 * (Math.sqrt(2) * sensors.getDistanceSideDist() +
                    (2 * Math.sqrt(2))); //S   C   A   L   E
            double target = init_height + s;
            int targetint = (int) Math.round(target);
            telemetry.addData("Target: ", targetint);
            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;

        raiseLiftTeleop(init_height);

    }
}
