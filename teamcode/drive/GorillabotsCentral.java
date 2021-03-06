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
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.DuckPipeline;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.RobotHardware;
import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components.Sensors;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import static java.lang.Math.abs;
import static java.lang.Math.max;

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
    public DuckPipeline PipelineD;
    public RobotHardware robot;
    public Pose2d duck_pos;

    public boolean intake_disabler = false;
    public boolean intake_disabler2 = false;

    public double lemniscate = 2.6220575542;

    OpenGLMatrix targetPose     = null;
    String targetName           = "";
    boolean targetFound     = false;    // Set to true when a target is detected by Vuforia
    double  targetRange     = 0;        // Distance from camera to target in Inches
    double  targetBearing   = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.

    public static int LIFT_CEILING = 2900;
    public static int LIFT_HIGH = 2750;
    public static int LIFT_SHARED = 1600; // 1700
    public static int LIFT_MID = 1780;
    public static int LIFT_LOW = 1100;
    public static int LIFT_BASE = 0;

    public final int intake_to_dist_period = 8;
    public int intake_to_dist_increment = 0;

    public static double LIFT_SPEED = .8;
    public static double OUTTAKE_TILT = .34;
    public static double OUTTAKE_UP = .46;
    public static double OUTTAKE_DOWN = 0.309;
    public static double OUTTAKE_SHARED = 0.1;
    public static int SHARED_HEIGHT = 1250;
    public static boolean LIFT_OVERRIDE = false;
    private static final String VUFORIA_KEY =
            "AYLQAH3/////AAABmSoSKoYZ0EQipFFJeSpGkIAAANdc2PvmNNHn5v9WrHajUc8Qkk7mqoScqSRhvoFnVNsJ87jyWWuoX+57RbBWL/Ns1SY5X/p/5PTX+PwSBoVHzKhjFoKrKLuFlBEQ/QZqP8ayH3j957ocoTcWV65KfgnwVmzvaYGKWrK5bo82RUQHKC8APD664jMeCi/A2jKfX4sxXwgl/DdPRtcYHq1JvQDDeaCA4R0YmRxtb5vE/KuB2Gfxwao2krsqRF9FibyZR1QkfaZQ2RVEnq0wzZ8q5kMBOjxbeVRw2yKjVESBZeUPHj66/YxkzdmEwQ5j8ERpToBsqZcwx/4w0kiecHztxzkqVemUoTn5e2CAAI1DpK7t";
    public final float CAMERA_FORWARD_DISPLACEMENT  = -17.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
    public final float CAMERA_VERTICAL_DISPLACEMENT = 15.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
    public final float CAMERA_LEFT_DISPLACEMENT     = 1.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

    public Pose2d offsetPose = new Pose2d();

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

    protected GorillabotsCentral() {
    }

    public void initializeComponents()
    {
        timer = new ElapsedTime();

        robot = new RobotHardware(hardwareMap, telemetry);

        sensors = new Sensors(hardwareMap,telemetry);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //servos = new Servos(hardwareMap, telemetry);

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        //Pipeline = new CVPipeline();
        //webcam.setPipeline(Pipeline);

        robot.outtake.setPosition(OUTTAKE_UP);

        }


    public void startVisionProcessing() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        Pipeline = new CVPipeline();
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
    }

    public void startDuckVision() {
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        PipelineD = new DuckPipeline();
        webcam.setPipeline(PipelineD);
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

    public void setLift(int position, double max_speed, double offset){
        int lift_init = robot.lift.getCurrentPosition();
        int lift_pos = lift_init;
        int target = position - robot.lift.getCurrentPosition();
        int error = target - lift_pos;
        double power = 0;

        while(Math.abs(error) > position * 0.01){ // > 100
            lift_pos = robot.lift.getCurrentPosition();
            if(error < 1){ offset =- offset; max_speed =- max_speed;}
            power = (error / target)  + offset;
            if(power > max_speed){ power = max_speed;}
            robot.lift.setPower(power);
        }
        robot.lift.setPower(0);
    }

    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        initializeComponents();
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

    public void realAutoDump(String level){
    final double LIFT_INIT = robot.lift.getCurrentPosition();
    double LIFT_POS = robot.lift.getCurrentPosition();
    double target = LIFT_INIT;
    switch (level){
        case "bottom":
            target = LIFT_LOW + LIFT_INIT;
            break;
        case "low":
            target = LIFT_LOW + LIFT_INIT;
            break;
        case "middle":
            target = LIFT_MID + LIFT_INIT;
            break;
        case "high":
            target = LIFT_HIGH + LIFT_INIT;
            break;
        case "bluelow":
            target = LIFT_LOW + LIFT_INIT + 150;
    }

    while(robot.lift.getCurrentPosition() < (target)){
        robot.lift.setPower(1);
    }
    robot.lift.setPower(0);
    sleep(300);
    robot.outtake.setPosition(OUTTAKE_DOWN *1.1);
    sleep(700);
    robot.outtake.setPosition(OUTTAKE_UP);
    sleep(200);
    double power = -1;
    final double peak = robot.lift.getCurrentPosition() + LIFT_INIT;
    while((robot.lift.getCurrentPosition() - LIFT_INIT) > -20 /*&& sensors.liftBot.getState()*/){


        power = -(Math.abs(robot.lift.getCurrentPosition() / peak)) - 0.35;

            robot.lift.setPower(power);

        telemetry.addData("sensor: ", sensors.liftBot.getState());
        telemetry.addData("pos: ", robot.lift.getCurrentPosition() - LIFT_INIT);
        telemetry.addData("power: ", power);
        telemetry.update();
        }
    robot.lift.setPower(0);
    //if above doesnt work, try this
        /*
        robot.lift.setPower(-1);
        boolean isLoop = true;
        while(isLoop){
        if(robot.lift.getCurrentPosition() - LIFT_INIT > 100){
        isLoop = false;
        robot.lift.setPower(0);
        }
        }
         */
    // or even try robot.lift.setMode(run to pos) method maybe research it
    }

    public void realAutoDumpRed(String level){
        final double LIFT_INIT = robot.lift.getCurrentPosition();
        double LIFT_POS = robot.lift.getCurrentPosition();
        double target = LIFT_INIT;
        switch (level){
            case "bottom":
                target = LIFT_LOW + LIFT_INIT;
                break;
            case "low":
                target = LIFT_LOW + LIFT_INIT;
                break;
            case "middle":
                target = LIFT_MID + LIFT_INIT;
                break;
            case "high":
                target = LIFT_HIGH + LIFT_INIT;
                break;
        }

        while(robot.lift.getCurrentPosition() < (target)){
            robot.lift.setPower(1);
        }
        robot.lift.setPower(0);
        sleep(300);
        robot.outtake.setPosition(OUTTAKE_DOWN *1.1);
        sleep(700);
        robot.outtake.setPosition(OUTTAKE_UP);
        sleep(200);
        double power = -1;
        final double peak = robot.lift.getCurrentPosition() + LIFT_INIT;
        while((robot.lift.getCurrentPosition() - LIFT_INIT) > -20 && sensors.liftBot.getState()){


            power = -(Math.abs(robot.lift.getCurrentPosition() / peak)) - 0.35;

            robot.lift.setPower(power);

            telemetry.addData("sensor: ", sensors.liftBot.getState());
            telemetry.addData("pos: ", robot.lift.getCurrentPosition() - LIFT_INIT);
            telemetry.addData("power: ", power);
            telemetry.update();
        }
        robot.lift.setPower(0);
        //if above doesnt work, try this
        /*
        robot.lift.setPower(-1);
        boolean isLoop = true;
        while(isLoop){
        if(robot.lift.getCurrentPosition() - LIFT_INIT > 100){
        isLoop = false;
        robot.lift.setPower(0);
        }
        }
         */
        // or even try robot.lift.setMode(run to pos) method maybe research it
    }

    public void lowerLift(){
        final double LIFT_INIT = robot.lift.getCurrentPosition();
        double LIFT_POS = robot.lift.getCurrentPosition();
        final double target = 0;
        while(sensors.liftBot.getState()){
            robot.lift.setPower(-1);
        }
        robot.lift.setPower(0);
    }

    public void autoDump(String level) {
        switch(level){
            case "bottom":
                robot.lift.setPower(.8);
                sleep(650);
                robot.lift.setPower(0);
                sleep(400);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(700);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(540);
                robot.lift.setPower(0);
                break;
            case "middle":
                robot.lift.setPower(.8);
                sleep(1100);
                robot.lift.setPower(0);
                sleep(400);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(700);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(750);
                robot.lift.setPower(0);
                break;
            case "top":
                robot.lift.setPower(.8);
                sleep(1500);
                robot.lift.setPower(0);
                sleep(400);
                robot.outtake.setPosition(OUTTAKE_DOWN);
                sleep(700);
                robot.outtake.setPosition(OUTTAKE_UP);
                sleep(200);
                robot.lift.setPower(-0.8);
                sleep(1200);
                robot.lift.setPower(0);
                break;
        }
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

    public String freightCheck() {
       // AsyncRev2MSensor asyncSensor = new AsyncRev2MSensor(sensors.dist);
        //asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_SPEED);

        if(sensors.dist.getDistance(DistanceUnit.INCH) >= 6.5){
            loadState = "NOTHING LOADED";
            intake_disabler = false;
            intake_disabler2 = false;
        }
        if(sensors.dist.getDistance(DistanceUnit.INCH) < 6.5){
            loadState = "LOADED";
            intake_disabler = true;
        }

        return loadState;
    }

    public void intakeToDist() {
        intake_to_dist_increment += 1;

        if(intake_to_dist_increment < intake_to_dist_period){
            robot.Intake1.setPower(1);
            robot.Intake2.setPower(-1);
        }
        else{
            intake_to_dist_increment = 0;
            if(freightCheck() == "NOTHING LOADED" && intake_disabler == false){
                if(intake_disabler == false) {
                    robot.Intake1.setPower(1);
                    robot.Intake2.setPower(-1);
                }
            }
            if(freightCheck() == "LOADED" || intake_disabler == true){
                robot.Intake1.setPower(0);
                robot.Intake2.setPower(0);
            }
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

    public void creepIntakeOffset(String direction, double timer){
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
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        offsetPose = drive.getPoseEstimate();
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
