package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;
//import org.opencv.core.Mat;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
@Config
public class liftTest extends GorillabotsCentral {// 192.168.43.1:8080/dash
    public static double kP = 0.1;
    public static double kI = 1;
    public static double kD = 4;
    @Override

    public void runOpMode() throws InterruptedException {
        initializeComponents();

        double target;

        double INIT_POS = robot.lift.getCurrentPosition();

        target = INIT_POS + 1800;

        double integralSum = 0;

        double error = 0;

        double lastError = 0;

        double derivative;

        boolean target_reached = true;

        double output;

        double MAX_SPEED = 0.75;

        double CURRENT_POS = INIT_POS;

        timer.reset();

        waitForStart();

        while(!isStopRequested()){

            //if(gamepad1.b){
            //    target += 500;
            //}

            CURRENT_POS = robot.lift.getCurrentPosition();

            error = target-robot.lift.getCurrentPosition();

            derivative = (error - lastError) / timer.seconds();

            integralSum = integralSum + (error * timer.seconds());

            output = (kP * error) + (kI * integralSum) + (kD * derivative);

            if(output > MAX_SPEED){
                output = MAX_SPEED;
            }
            if(target_reached){
                output = 0;
            }

            robot.lift.setPower(output);

            lastError = error;

            if(((CURRENT_POS < target * .95) || CURRENT_POS > target  *1.15)){
                target_reached = false;
            }
            if(((CURRENT_POS > target * 0.95) && CURRENT_POS < target * .85)){
                target_reached = true;
                robot.lift.setPower(0);
            }
            if(error < 50 && error > -50){
                target_reached = true;
                robot.lift.setPower(0);
            }

            telemetry.addData("Current position: ", CURRENT_POS);
            telemetry.addData("Target: ", target);
            telemetry.addData("Error: ", error);
            telemetry.addData("Output Speed: ", output);
            telemetry.addData("Target Reached: ", target_reached);
            telemetry.update();
            timer.reset();

        }

    }
}
