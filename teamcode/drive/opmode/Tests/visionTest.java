package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.opmode.Tests;


import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.CVPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


@TeleOp(group = "AAAAAAAAAA", name = "visionTest")

public class visionTest extends LinearOpMode {
    public void runOpMode() {

        // 192.168.43.1:8080/dash

        FtcDashboard dashboard = FtcDashboard.getInstance();

        CameraStreamSource src = new CameraStreamSource() {
            @Override
            public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

            }
        };

        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        CVPipeline Pipeline;

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        Pipeline = new CVPipeline();
        webcam.setPipeline(Pipeline);

        webcam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webcam, 24);

        webcam.setPipeline(Pipeline);

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

        while (!isStarted() && !isStopRequested()){
            dashboardTelemetry.addData("Position: ", Pipeline.getAnalysis());
            dashboardTelemetry.addData("Avg1: ", Pipeline.getAvg1());
            dashboardTelemetry.addData("Avg2: ", Pipeline.getAvg2());
            dashboardTelemetry.addData("Avg3: ", Pipeline.getAvg3());
            dashboardTelemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();
        }

    }

}