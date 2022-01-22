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
public class DriveTest extends GorillabotsCentral { // 192.168.43.1:8080/dash

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

        waitForStart();

        while (!isStopRequested()) {

            if(gamepad1.a && drive_timer.time() >= 0.4){
                drive_timer.reset();
                drive_check =! drive_check;
            }

            if(drive_check){
                drive_state = "gap";
            }
            if(!drive_check){
                drive_state = "normal";
            }

            switch(drive_state){
                case "normal":
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                    break;
                case "gap":
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x - 0.35,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
            }

            telemetry.addData("Drive state: ", drive_state);
            telemetry.update();

        }
    }

}


