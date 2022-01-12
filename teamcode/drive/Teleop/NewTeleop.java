package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;

@TeleOp(group="main", name="NewTeleop")
@Disabled
@Config
public class NewTeleop extends GorillabotsCentral {


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    String drive_state = "normal";

    ElapsedTime timer = new ElapsedTime();

    final double NORMAL_SPEED_CONSTANT = .9;
    final double INTER_SPEED_CONSTANT = .5;
    final double SLOW_SPEED_CONSTANT = .3;

    public static double LIFT_SPEED_MULTIPLIER = .7;
    public double LIFT_POS = 0;

    public String Lift_state = "stop";


    String swivel_state = "mid";

    final double INTAKE_SPEED = 1;
    final double SPINNER_SPEED = .80;

    final double SWIVEL_LEFT = .91;
    final double SWIVEL_RIGHT = .01;

    public DcMotor intake;

    public DcMotor spinner;

    public String intake_toggle = "off";

    public DcMotor lift;

    public Servo swivel;

    @Override
    public void runOpMode() {

        initializeComponents();

        ElapsedTime swiveltime = new ElapsedTime();

        ElapsedTime intake_time = new ElapsedTime();

        intake = hardwareMap.dcMotor.get("intake");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinner = hardwareMap.dcMotor.get("spinner");

        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = hardwareMap.dcMotor.get("lift");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        swivel = hardwareMap.servo.get("swivel");

        double LIFT_CEILING = lift.getCurrentPosition() -3650;

        waitForStart();

        while (opModeIsActive()) {

            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double r = -gamepad1.right_stick_x;

            LIFT_POS = lift.getCurrentPosition();

            if (gamepad1.a && timer.time() >= .4) {
                drive_state = "normal";
                timer.reset();
            }
            if(gamepad1.left_trigger > .4){
                spinner.setPower(SPINNER_SPEED);
            }
            if(gamepad1.right_trigger >.4){
                spinner.setPower(-SPINNER_SPEED);
            }
            if(gamepad1.right_trigger < .4 && gamepad1.left_trigger <.4){
                spinner.setPower(0);
            }

            if (gamepad1.b && timer.time() >= .4) {
                drive_state = "slow";
                timer.reset();
            }

            if (gamepad1.y && timer.time() >= .4) {
                drive_state = "inter";
                timer.reset();
            }

            if (gamepad1.left_bumper && intake_time.time() >= 0.8){
                if(intake_toggle == "on"){
                    intake_toggle = "off";
                    intake_time.reset();
                }
                else {
                    intake_toggle = "on";
                    intake_time.reset();
                    telemetry.addLine("Intake on");
                }
            }

            if (gamepad1.right_bumper && intake_time.time() >= 0.8){
                if(intake_toggle == "reverse"){
                    intake_toggle = "off";
                    intake_time.reset();
                }
                else {
                    intake_toggle = "reverse";
                    intake_time.reset();
                    telemetry.addLine("Intake reversed");
                }
            }

            if(gamepad2.right_trigger >= .2 && gamepad2.left_trigger <= .5 && -LIFT_POS - LIFT_CEILING > LIFT_CEILING){
                Lift_state = "up";
            }

            if(gamepad2.left_trigger >= .2 && gamepad2.right_trigger <= .2){
                Lift_state = "down";
            }

            if((gamepad2.left_trigger <= .2 && gamepad2.right_trigger <= .2) || -LIFT_POS - LIFT_CEILING <= LIFT_CEILING && Lift_state != "down"){
                Lift_state = "stop";
            }

            if(gamepad2.x && swiveltime.time() >= 1){
                swivel_state = "left";
                telemetry.addLine("Swivel Left");
                swiveltime.reset();
            }

            //if(gamepad2.right_stick_button && gamepad2.left_stick_button){
              //  LIFT_CEILING = lift.getCurrentPosition() - 3650;
            //}

            if(gamepad2.b && swiveltime.time() >= 1){
                swivel_state = "right";
                telemetry.addLine("Swivel Right");
                swiveltime.reset();
            }

            if(gamepad2.a && swiveltime.time() >= 1) {
                swivel_state = "mid";
                telemetry.addLine("Swivel Mid");
                swiveltime.reset();
            }

            switch (Lift_state){
                case "stop":
                    lift.setPower(0);
                    break;
                case "down":
                    lift.setPower(-gamepad2.left_trigger * LIFT_SPEED_MULTIPLIER);
                    break;
                case "up":
                    lift.setPower(gamepad2.right_trigger * LIFT_SPEED_MULTIPLIER);
            }



            switch (intake_toggle) {
                case ("off"):
                    intake.setPower(0);
                    dashboardTelemetry.addLine("Intake Off");
                    break;

                case ("on"):
                    intake.setPower(INTAKE_SPEED);
                    dashboardTelemetry.addLine("Intake On");
                    break;

                case ("reverse"):
                    intake.setPower(-INTAKE_SPEED);
                    dashboardTelemetry.addLine("Intake Reversed");
                    break;
            }

            switch (drive_state) {
                case "normal":
                    drive.go(x * NORMAL_SPEED_CONSTANT, y * NORMAL_SPEED_CONSTANT, r * NORMAL_SPEED_CONSTANT);
                    break;
                case "inter":
                    drive.go(x * INTER_SPEED_CONSTANT, y * INTER_SPEED_CONSTANT, r * SLOW_SPEED_CONSTANT);
                    break;
                case "slow":
                    drive.go(x * SLOW_SPEED_CONSTANT, y * SLOW_SPEED_CONSTANT, r * SLOW_SPEED_CONSTANT);
                    break;
            }


            telemetry.addData("Drive State", drive_state);
            telemetry.addData("Ceiling: ",LIFT_CEILING);
            telemetry.addData("CurPos: ", lift.getCurrentPosition());
            telemetry.update();
        }
    }
}