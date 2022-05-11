package org.firstinspires.ftc.teamcodeGIT.teamcode.drive.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcodeGIT.teamcode.drive.GorillabotsCentral;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class DuckPipeline extends OpenCvPipeline
{

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    /*
     * An enum to define the skystone position
     */
    public enum BarcodePosition
    {
        LEFT_OUT,
        LEFT_IN,
        RIGHT_IN,
        RIGHT_OUT
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    Scalar NEON_GREEN = new Scalar(30, 227, 39);

    // Pink, the default color                         Y      Cr     Cb    (Do not change Y)
    public static Scalar scalarLowerYCrCb = new Scalar(198, 25, 128);
    public static Scalar scalarUpperYCrCb = new Scalar(73, 95, 91);

    /*
     * The core values which define the location and size of the sample regions
     */

    //1280 by 720
    static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(5,300);
    static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(325,300);
    static Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(645,300);
    static Point REGION4_TOPLEFT_ANCHOR_POINT = new Point(965,300);
    static int REGION_WIDTH = 310;
    static int REGION_HEIGHT = 400;


    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region4_pointA = new Point(
            REGION4_TOPLEFT_ANCHOR_POINT.x,
            REGION4_TOPLEFT_ANCHOR_POINT.y);
    Point region4_pointB = new Point(
            REGION4_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION4_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cb, region2_Cb, region3_Cb, region4_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2, avg3, avg4;

    private volatile BarcodePosition position = BarcodePosition.LEFT_OUT;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame)
    {

        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        region4_Cb = Cb.submat(new Rect(region4_pointA, region4_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {

        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg3 = (int) Core.mean(region3_Cb).val[0];
        avg4 = (int) Core.mean(region4_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 4 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region4_pointA, // First point which defines the rectangle
                region4_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        int max = 0;

        if(avg1 < avg2 && avg1 < avg3 && avg1 < avg4){
            max = 1;
        }
        if(avg2 < avg1 && avg2 < avg3 && avg2 < avg4){
            max = 2;
        }
        if(avg3 < avg1 && avg3 < avg2 && avg3 < avg4){
            max = 3;
        }
        if(avg4 < avg1 && avg4 < avg2 && avg4 < avg3){
            max = 4;
        }

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(max == 1) // Was it from region 1?
        {
            position = BarcodePosition.LEFT_OUT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == 2) // Was it from region 2?
        {
            position = BarcodePosition.LEFT_IN; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == 3) // Was it from region 3?
        {
            position = BarcodePosition.RIGHT_IN; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == 4) // Was it from region 3?
        {
            position = BarcodePosition.RIGHT_OUT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region4_pointA, // First point which defines the rectangle
                    region4_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public BarcodePosition getAnalysis()
    {
        return position;
    }
    public int getPos(){
        int pos = 0;
        if (position == BarcodePosition.LEFT_OUT){
            pos = 1;
        }
        else if (position == BarcodePosition.LEFT_IN){
            pos = 2;
        }
        else if (position == BarcodePosition.RIGHT_IN){
            pos = 3;
        }
        else if (position == BarcodePosition.RIGHT_OUT){
            pos = 4;
        }
        return pos;
    }
    public Pose2d duck_pos(){
        Pose2d poop = new Pose2d();
        if (position == BarcodePosition.LEFT_OUT){
            poop = new Pose2d(-45, -65, Math.toRadians(90));
        }
        if (position == BarcodePosition.LEFT_IN){
            poop = new Pose2d(-48, -65, Math.toRadians(90));
        }
        if(position == BarcodePosition.RIGHT_IN){
            poop = new Pose2d(-51, -65, Math.toRadians(90));
        }
        if(position == BarcodePosition.RIGHT_OUT){
            poop = new Pose2d(-43.5, -65, Math.toRadians(80));
        }
        return poop;
    }
    public int getAvg1(){
        return avg1;
    }
    public int getAvg2(){
        return avg2;
    }
    public int getAvg3(){
        return avg3;
    }
    public int getAvg4(){
        return avg4;
    }
}