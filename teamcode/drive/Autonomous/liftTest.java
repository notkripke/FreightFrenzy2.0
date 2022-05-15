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

        float kP = 0;
        float kI = 0;
        float kD = 0;

        double target;

        double INIT_POS = robot.lift.getCurrentPosition();

        target = INIT_POS;

        double integralSum = 0;

        double error = 0;

        double lastError = 0;

        double derivative;

        boolean target_reached = true;

        double output;

        double MAX_SPEED = 0.4;

        double CURRENT_POS = INIT_POS;

        timer.reset();

        waitForStart();

        while(!isStopRequested()){

            if(gamepad1.b){
                target += 500;
            }

            CURRENT_POS = robot.lift.getCurrentPosition();

            error = target-robot.lift.getCurrentPosition();

            derivative = (error - lastError) / timer.seconds();

            integralSum = integralSum + (error * timer.seconds());

            output = (kP * error) + (kI * integralSum) + (kD * derivative);

            if(output > MAX_SPEED){
                output = MAX_SPEED;
            }

            robot.lift.setPower(output);

            lastError = error;

            if(((CURRENT_POS < target * .95) || CURRENT_POS > target  *1.05)){
                target_reached = false;
            }
            if(((CURRENT_POS > target * 0.95) && CURRENT_POS < target * 1.05)){
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
