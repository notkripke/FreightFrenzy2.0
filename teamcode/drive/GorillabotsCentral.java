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