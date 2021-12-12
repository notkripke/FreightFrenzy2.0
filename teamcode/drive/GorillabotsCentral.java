package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.Components.CVPipeline;
import org.firstinspires.ftc.teamcode.drive.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.Components.Sensors;
import org.firstinspires.ftc.teamcode.drive.Components.Servos;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static java.lang.Math.abs;

//import org.firstinspires.ftc.teamcode.components.RevGyro;

@Disabled
public abstract class GorillabotsCentral extends LinearOpMode {

    public Sensors sensors;
    public SampleMecanumDrive drive;
    public ElapsedTime timer;
    public OpenCvCamera webcam;
    public Servos servos;
    public CVPipeline Pipeline;
    public RobotHardware robot;


    public void initializeComponents()
    {
        timer = new ElapsedTime();

        robot = new RobotHardware(hardwareMap, telemetry);

        sensors = new Sensors(hardwareMap,telemetry);

        drive = new SampleMecanumDrive(hardwareMap);

        servos = new Servos(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        Pipeline = new CVPipeline();
        webcam.setPipeline(Pipeline);
        };


    final double INTAKE_SPEED = 1;
    final double SPINNER_SPEED = .80;

    final double SWIVEL_LEFT = .91;
    final double SWIVEL_RIGHT = .06;
    final double NORMAL_SPEED_CONSTANT = .9;
    final double INTER_SPEED_CONSTANT = .5;
    final double SLOW_SPEED_CONSTANT = .3;

    public static double CRAWL_SPEED_CONSTANT = .08;
    public static double LIFT_SPEED_MULTIPLIER = 1;
    public int LIFT_HEIGHT1 = 350;
    public int LIFT_HEIGHT2 = 3600;
    public double LIFT_POS = 0;

    public void raiseLift1(){//REMEMBER: For ALL lift functions, add sleep() times in auto
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() + LIFT_HEIGHT1);
        robot.lift.setPower(.6);
    }
    public void raiseLift2(){
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() + LIFT_HEIGHT2);
        robot.lift.setPower(.6);
    }

    public void raiseLiftVar(int height, double speed){
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        height = (height/100) * 4200;

        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() + height);
        robot.lift.setPower(speed);
    }

    public void  lowerLift(){
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() - (LIFT_HEIGHT1 +LIFT_HEIGHT2));
    }

    public void intake(int time){
        robot.intake.setPower(INTAKE_SPEED);
        sleep(time);
        robot.intake.setPower(0);
    }
    public void reverseIntake(int time){
        robot.intake.setPower(-INTAKE_SPEED);
        sleep(time);
        robot.intake.setPower(0);
    }
    public void stopIntake(){
        robot.intake.setPower(0);
    }

    public void spinWheel(String side){
        if (side == "red"){
            robot.spinner.setPower(SPINNER_SPEED);
            sleep(2500);
            robot.spinner.setPower(0);
        }
        else if(side == "blue"){
            robot.spinner.setPower(-SPINNER_SPEED);
            sleep(2500);
            robot.spinner.setPower(0);
        }
    }

    public void swivel(String direction){ //REMEMBER: use sleep() for use in autonomous
        if(direction == "left"){
            robot.swivel.setPosition(SWIVEL_LEFT);
        }
        else if (direction == "right"){
            robot.swivel.setPosition(SWIVEL_RIGHT);
        }
        else if (direction == "mid"){
            robot.swivel.setPosition(.5);
        }
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

    public void stopVisionProcessing(){
        webcam.stopStreaming();
    }




}