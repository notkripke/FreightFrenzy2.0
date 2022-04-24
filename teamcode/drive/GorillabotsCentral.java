package org.firstinspires.ftc.teamcodeGIT.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Output;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.CVPipeline;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.RobotHardware;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.Sensors;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor.AsyncRev2MSensor;

import static java.lang.Math.abs;

import java.util.ArrayList;
import java.util.List;

//import org.firstinspires.ftc.teamcodeGIT.teamcode.components.RevGyro;

@Config
public abstract class GorillabotsCentral extends LinearOpMode {//testing

    public Sensors sensors;
    public SampleMecanumDrive drive;
    public ElapsedTime timer;
    public OpenCvCamera webcam;
    //public Servos servos;
    public CVPipeline Pipeline;
    public RobotHardware robot;

    public double lemniscate = 2.6220575542;

    OpenGLMatrix targetPose     = null;
    String targetName           = "";
    boolean targetFound     = false;    // Set to true when a target is detected by Vuforia
    double  targetRange     = 0;        // Distance from camera to target in Inches
    double  targetBearing   = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
    public static int LIFT_CEILING = 2430;

    public static double LIFT_SPEED = .8;
    public static double OUTTAKE_TILT = .36;
    public static double OUTTAKE_UP = .315;
    public static double OUTTAKE_DOWN = 0.18;
    public static int SHARED_HEIGHT = 1250;
    public static boolean LIFT_OVERRIDE = false;
    private static final String VUFORIA_KEY =
            " --- YOUR NEW VUFORIA KEY GOES HERE  --- ";
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


    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public VuforiaTrackables targets   = null ;


    public void VuforiaScan(double time, boolean activated){
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
        while (!isStopRequested() && (vuforiaclock.time() <= time || activated)) {

            // check all the trackable targets to see which one (if any) is visible.
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
        }

        // Disable Tracking when we are done;
        targets.deactivate();
    }

    public void resetVuforiaParameters(){ //call when done with vision
        targetFound = false;
        targetBearing = 0;
        targetName = "";
        targetRange = 0;
        targetPose = null;
    }

    public Pose2d VuforiaLocalizedPose2d(){
        return new Pose2d(vuforiaScanPos_X, vuforiaScanPos_Y, Math.toRadians(vuforiaScanPos_HEADING));
    }

    public void LED(String leds){

        if(leds == "all"){
            sensors.blLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.brLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.flLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.frLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.flLED.setState(true);
            sensors.frLED.setState(true);
            sensors.blLED.setState(true);
            sensors.brLED.setState(true);
        }

        else if(leds == "back"){
            sensors.blLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.brLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.flLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.frLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.brLED.setState(true);
            sensors.blLED.setState(true);
        }
        else if(leds == "front"){
            sensors.blLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.brLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.flLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.frLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.frLED.setState(true);
            sensors.flLED.setState(true);
        }

        else if(leds == "left"){
            sensors.blLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.brLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.flLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.frLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.flLED.setState(true);
            sensors.blLED.setState(true);
        }
        else if(leds == "right"){
            sensors.brLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.frLED.setMode(DigitalChannel.Mode.OUTPUT);
            sensors.blLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.flLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.frLED.setState(true);
            sensors.brLED.setState(true);
        }
        else if(leds == "none" || leds == "off"){
            sensors.blLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.brLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.flLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.frLED.setMode(DigitalChannel.Mode.INPUT);
            sensors.frLED.setState(false);
            sensors.flLED.setState(false);
            sensors.brLED.setState(false);
            sensors.blLED.setState(false);
        }
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
        while((robot.lift.getCurrentPosition() < targetint) && (target < 2450) && !LIFT_OVERRIDE){
            robot.outtake.setPosition(OUTTAKE_UP);
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
        AsyncRev2MSensor asyncSensor = new AsyncRev2MSensor(sensors.dist);
        asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_SPEED);

        if(asyncSensor.getDistance(DistanceUnit.INCH) >= 5.5){
            loadState = "NOTHING LOADED";

        }
        if(asyncSensor.getDistance(DistanceUnit.INCH) < 5.5){
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
