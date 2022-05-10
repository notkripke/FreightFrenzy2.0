package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.opmode.Tests;


import android.graphics.Bitmap;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.CVPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import java.util.ArrayList;
import java.util.List;


@TeleOp(group = "AAAAAAAAAA", name = "IsolatedVuforiaTest")

public class IsolatedVuforiaTest extends LinearOpMode {

    public VuforiaTrackables targets   = null ;
    OpenGLMatrix targetPose     = null;
    String targetName           = "";
    boolean targetFound     = false;    // Set to true when a target is detected by Vuforia
    double  targetRange     = 0;        // Distance from camera to target in Inches
    double  targetBearing   = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
    private static final String VUFORIA_KEY =
            "AYLQAH3/////AAABmSoSKoYZ0EQipFFJeSpGkIAAANdc2PvmNNHn5v9WrHajUc8Qkk7mqoScqSRhvoFnVNsJ87jyWWuoX+57RbBWL/Ns1SY5X/p/5PTX+PwSBoVHzKhjFoKrKLuFlBEQ/QZqP8ayH3j957ocoTcWV65KfgnwVmzvaYGKWrK5bo82RUQHKC8APD664jMeCi/A2jKfX4sxXwgl/DdPRtcYHq1JvQDDeaCA4R0YmRxtb5vE/KuB2Gfxwao2krsqRF9FibyZR1QkfaZQ2RVEnq0wzZ8q5kMBOjxbeVRw2yKjVESBZeUPHj66/YxkzdmEwQ5j8ERpToBsqZcwx/4w0kiecHztxzkqVemUoTn5e2CAAI1DpK7t";
    public final float CAMERA_FORWARD_DISPLACEMENT  = -17.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
    public final float CAMERA_VERTICAL_DISPLACEMENT = 15.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
    public final float CAMERA_LEFT_DISPLACEMENT     = 1.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens


    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    public static final float halfField        = 72 * mmPerInch;
    public static final float halfTile         = 12 * mmPerInch;
    public static final float oneAndHalfTile   = 36 * mmPerInch;
    public OpenGLMatrix lastLocation   = null;
    public VuforiaLocalizer vuforia    = null;
    public boolean targetVisible       = false;

    public float vuforiaScanPos_X;
    public float vuforiaScanPos_Y;
    public float vuforiaScanPos_HEADING; // In Degrees

    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public void runOpMode() {

        // 192.168.43.1:8080/dash

        FtcDashboard dashboard = FtcDashboard.getInstance();

        CameraStreamSource src = new CameraStreamSource() {
            @Override
            public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

            }
        };

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElapsedTime vuforiaclock = new ElapsedTime();
        vuforiaclock.reset();
        OpenGLMatrix lastLocation   = null;
        VuforiaLocalizer vuforia    = null;
        WebcamName webcamName       = null;
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.useExtendedTracking = false;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));



        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }


        targets.activate();

        waitForStart();
        while (opModeIsActive()) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",  translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                vuforiaScanPos_X = translation.get(0) / mmPerInch;
                vuforiaScanPos_Y = translation.get(1) / mmPerInch;

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                vuforiaScanPos_HEADING = rotation.thirdAngle + 180;

            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();




            telemetry.update();
        }
        targets.deactivate();

    }

}