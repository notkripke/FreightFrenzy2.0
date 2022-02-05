package org.firstinspires.ftc.teamcodeGIT.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.CVPipeline;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.RobotHardware;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.Sensors;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static java.lang.Math.abs;

//import org.firstinspires.ftc.teamcodeGIT.teamcode.components.RevGyro;

@Config
public abstract class GorillabotsCentral extends LinearOpMode {

    public Sensors sensors;
    public SampleMecanumDrive drive;
    public ElapsedTime timer;
    public OpenCvCamera webcam;
    //public Servos servos;
    public CVPipeline Pipeline;
    public RobotHardware robot;

    public double lemniscate = 2.6220575542;

    public static int LIFT_CEILING = 2430;
    public static double LIFT_SPEED = .8;
    public static double OUTTAKE_UP = .35;
    public static double OUTTAKE_DOWN = 0.01;
    public static int SHARED_HEIGHT = 1250;
    public static boolean LIFT_OVERRIDE = false;

    public String loadState = "NOTHING LOADED";

    public void initializeComponents()
    {
        timer = new ElapsedTime();

        robot = new RobotHardware(hardwareMap, telemetry);

        sensors = new Sensors(hardwareMap,telemetry);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //servos = new Servos(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        Pipeline = new CVPipeline();
        webcam.setPipeline(Pipeline);

        robot.outtake.setPosition(OUTTAKE_UP);

        }


    public void startVisionProcessing() {
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
    }

    public void raiseLift(int net_height, double speed){
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        final int lift_target = net_height + robot.lift.getCurrentPosition();

        robot.lift.setTargetPosition(lift_target);
        while(robot.lift.getCurrentPosition() < (lift_target * 0.8)){
            robot.lift.setPower(speed);
            telemetry.addData("lift height: ", robot.lift.getCurrentPosition());
        }
        while(robot.lift.getCurrentPosition() < lift_target){
            robot.lift.setPower(speed * 0.75);
            telemetry.addData("lift height: ", robot.lift.getCurrentPosition());
        }
        robot.lift.setPower(0);
        telemetry.update();
    }

    public void lowerLift(double speed,int height_estimate){
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final int target = robot.lift.getCurrentPosition() - height_estimate;

        while(robot.lift.getCurrentPosition() > (target * .65) /*&& sensors.checkSwitch() == false*/){
            robot.lift.setPower(-speed);
        }
        while(robot.lift.getCurrentPosition() > (target * .92) /*&& sensors.checkSwitch() == false*/){
            robot.lift.setPower(-speed * 0.68);
        }
        robot.lift.setPower(0);
    }

    public void raiseLiftTeleop(int init_height){
        double s = 94*(Math.sqrt(2) * sensors.getDistanceSideDist()+ (2*Math.sqrt(2))); //S   C   A   L   E
        double target = init_height + s +150;
        int targetint = (int) Math.round(target);
        robot.lift.setTargetPosition(targetint);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.outtake.setPosition(OUTTAKE_UP);
        while((robot.lift.getCurrentPosition() < targetint) && (target < 2450) && !LIFT_OVERRIDE){
            if(gamepad2.left_trigger > .2 || gamepad2.right_trigger > .2 || gamepad2.y){
               LIFT_OVERRIDE = true;
            }
            robot.lift.setPower(0.85);
            telemetry.addData("target: ", targetint);
            telemetry.addData("currpos: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }
        robot.lift.setPower(0);
        robot.outtake.setPosition(OUTTAKE_DOWN);
    }

    public String freightCheck() {
        if(sensors.dist.getDistance(DistanceUnit.INCH) >= 3.5){
            loadState = "NOTHING LOADED";
        }
        if(sensors.dist.getDistance(DistanceUnit.INCH) < 3.5){
            loadState = "LOADED";
        }

        return loadState;
    }

    public void intakeToDist() {
        if(freightCheck() == "NOTHING LOADED"){
            robot.Intake1.setPower(1);
            robot.Intake2.setPower(-1);
        }
        if(freightCheck() == "LOADED"){
            robot.Intake1.setPower(0);
            robot.Intake2.setPower(0);
        }
    }

    public void creepIntake(String direction, double timer){
        ElapsedTime intake_timer = new ElapsedTime();
        intake_timer.reset();
        while(freightCheck() == "NOTHING LOADED" && intake_timer.milliseconds() <= timer){
            robot.Intake1.setPower(1);
            robot.Intake2.setPower(-1);
            if(direction == "forwards"){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0.18,
                                0,
                                0
                        )
                );
                drive.update();
            }
            if(direction == "backwards"){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -0.18,
                                0,
                                0
                        )
                );
                drive.update();
            }
        }
        robot.Intake2.setPower(0);
        robot.Intake1.setPower(0);
    }

    public void stopVisionProcessing(){
        webcam.stopStreaming();
    }

    public double cosh(double x){
        return (Math.exp(x) + Math.exp(-x))/2;
    };

    public double sinlemn(double x){
        double[] storage = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        for(int n = 0; n <= 10; n++){
            storage[n] = Math.sin(Math.PI*x*(2*n+1)/lemniscate)/((2*n+1)*cosh(Math.PI*(n+0.5)));
        }

        double sum = 0;

        for(int i = 0; i < storage.length; i++){
            sum += storage[i];
        }

        return Math.tan(2*sum);
    }
}
