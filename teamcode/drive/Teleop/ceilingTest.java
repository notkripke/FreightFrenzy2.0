package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.StandardTrackingWheelLocalizer;


@TeleOp(group = "drive")
@Config
public class ceilingTest extends GorillabotsCentral { // 192.168.43.1:8080/dash

    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // telemetry uses
        // FTC app and Dashboard
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        ElapsedTime drive_timer = new ElapsedTime();

        boolean drive_check = false;

        String drive_state = "normal";

        String Lift_state = "stop";

        ElapsedTime duckPower = new ElapsedTime();

        String duck_trigger = "off";

        final double LIFT_INIT = robot.lift.getCurrentPosition();
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double tracker = 0;

        double LIFT_POS;

        waitForStart();

        while (!isStopRequested()) {

            LIFT_POS = robot.lift.getCurrentPosition();

            if(gamepad2.left_trigger >= .2 && gamepad2.right_trigger <= .2 && sensors.liftBot.getState()){//liftpos - ceiling < ceiling
                Lift_state = "down";
            }
            if(gamepad2.right_trigger >= .2 && gamepad2.left_trigger <= .2 ){
                Lift_state = "up";
            }
            if(gamepad2.left_trigger <= .2 && gamepad2.right_trigger <= .2){
                Lift_state = "stop";
            }

            switch (Lift_state){
                case "stop":
                    robot.lift.setPower(0);
                    break;
                case "down":
                    if(sensors.liftBot.getState()) {
                        robot.lift.setPower(-gamepad2.left_trigger);
                    }
                    if(!sensors.liftBot.getState()){
                        robot.lift.setPower(0);
                    }
                    break;
                case "up":
                    robot.lift.setPower(gamepad2.right_trigger);
                    break;
            }
            telemetry.addData("lift height", LIFT_POS);
            telemetry.addData("liftInit", LIFT_INIT);
            telemetry.addData("difference", LIFT_POS-LIFT_INIT);
            telemetry.update();

        }

    }
}



