package org.firstinspires.ftc.teamcodeGIT.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    
    //peepee poopoo 

    public Sensors sensors;
    public SampleMecanumDrive drive;
    public ElapsedTime timer;
    public OpenCvCamera webcam;
    //public Servos servos;
    public CVPipeline Pipeline;
    public RobotHardware robot;

    public static int LIFT_CEILING = 2300;
    public static double LIFT_SPEED = .8;
    public static double OUTTAKE_UP = .025;
    public static double OUTTAKE_DOWN = 0;
    public static int SHARED_HEIGHT = 1250;


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
        final int lift_target = net_height + robot.lift.getCurrentPosition();
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
    public void stopVisionProcessing(){
        webcam.stopStreaming();
    }




}
