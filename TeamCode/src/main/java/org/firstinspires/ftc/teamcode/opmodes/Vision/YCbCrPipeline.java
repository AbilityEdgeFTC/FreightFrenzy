/**
 * Created by Ability Edge#18273
 * - Elior Yousefi, and Eitan Kravets
 */
package org.firstinspires.ftc.teamcode.opmodes.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class YCbCrPipeline extends OpenCvPipeline
{

    public static double lowValuesRedY = 73;
    public static double highValuesRedY = 113;
    public static double lowValuesRedB = 176;
    public static double highValuesRedB = 220;
    public static double lowValuesRedR = 96;
    public static double highValuesRedR = 116;
    public static double lowValuesBlueY = 0;
    public static double highValuesBlueY = 221;
    public static double lowValuesBlueB = 99;
    public static double highValuesBlueB = 139;
    public static double lowValuesBlueR = 134;
    public static double highValuesBlueR = 190;
    // creating a mast with the same resolution of the webcam for the place to display the detected team shipping element
    Mat mask = new Mat(1280,720,0);//
    Mat inputYCbCr = new Mat(1280,720,0);

    // creating 3 rectangles(sections) for checking the colors inside them.
    final Rect LEFT_SEC = new Rect(
            new Point(426.666667,300),//mask.cols()/7, mask.rows()/5 * 2
            new Point(0,720));//mask.cols()/7 * 2, mask.rows()/5 * 4)


    final Rect CENTER_SEC = new Rect(
            new Point(853.333334,300),
            new Point(426.666667,720));

    final Rect RIGHT_SEC = new Rect(
            new Point(1280,300),
            new Point(853.333334,720));

    public static boolean DEBUG = true;

    public static double threshold = 10000;

    // the list of locations that can be
    public enum Location{
        Left,
        Center,
        Right,
        Not_Found
    }

    public Location location;

    // color for the rectangles to show on the screen
    Scalar colorBarcodeRect = new Scalar(0, 255, 0);

    public Telemetry telemetry;

    public static boolean redAlliance = true;

    boolean barcodeLeft, barcodeCenter, barcodeRight;

    @Override
    public Mat processFrame(Mat input) {
        // HSV low and high values for our team shipping element.
        Scalar lowValuesRED = new Scalar(lowValuesRedY, lowValuesRedB, lowValuesRedR);
        Scalar highValuesRED = new Scalar(highValuesRedY, highValuesRedB, highValuesRedR);
        Scalar lowValuesBLUE = new Scalar(lowValuesBlueY, lowValuesBlueB, lowValuesBlueR);
        Scalar highValuesBLUE = new Scalar(highValuesBlueY, highValuesBlueB, highValuesBlueR);

        Imgproc.cvtColor(input, inputYCbCr, Imgproc.COLOR_RGB2YCrCb);

        // turning all colors not between the low and high values to black and the rest white.
        if(redAlliance)
        {
            Core.inRange(inputYCbCr, lowValuesRED, highValuesRED, mask);
        }
        else
        {
            Core.inRange(inputYCbCr, lowValuesBLUE, highValuesBLUE, mask);
        }

        // taking sections from the mask to another mat
        Mat left = mask.submat(LEFT_SEC);
        Mat center = mask.submat(CENTER_SEC);
        Mat right = mask.submat(RIGHT_SEC);

        barcodeLeft = Core.countNonZero(left) > threshold;
        barcodeCenter = Core.countNonZero(center) > threshold;
        barcodeRight = Core.countNonZero(right) > threshold;

        // we release the mats for use.
        left.release();
        center.release();
        right.release();

        //checking which barcode is found.
        if(barcodeLeft && barcodeCenter && barcodeRight){
            // NOT FOUND
            setLocation(Location.Not_Found);
        }else if(barcodeLeft){
            setLocation(Location.Left);
        }else if(barcodeCenter){
            setLocation(Location.Center);
        }else if(barcodeRight){
            setLocation(Location.Right);
        }else{
            // NOT FOUND
            setLocation(Location.Not_Found);
        }

        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);

        // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
        Imgproc.rectangle(mask, LEFT_SEC, colorBarcodeRect);
        Imgproc.rectangle(mask, CENTER_SEC, colorBarcodeRect);
        Imgproc.rectangle(mask, RIGHT_SEC, colorBarcodeRect);

        return mask;
    }

    // getting location of the team shipping element.
    public Location getLocation() {
        return location;
    }

    public void setLocation(Location location) {
        this.location = location;
    }

    public static boolean isRedAlliance() {
        return redAlliance;
    }

    public static void setRedAlliance(boolean redAlliance) {
        YCbCrPipeline.redAlliance = redAlliance;
    }

    public void setTelementry(Telemetry telemetry){ this.telemetry = telemetry; }
}