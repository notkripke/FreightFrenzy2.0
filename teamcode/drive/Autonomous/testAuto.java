package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;


import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;
import org.opencv.core.Mat;
/*
 * This is an example of a more complex path to really test the tuning.
 */

@Autonomous(group = "drive")
@Config
public class testAuto extends GorillabotsCentral {



    static double speed = 0.70;
    static int height = 2299;

    static double lower_speed = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        waitForStart();

        raiseLift(height, speed);
        sleep(2000);
        lowerLift(lower_speed, height - 50);

    }
}
