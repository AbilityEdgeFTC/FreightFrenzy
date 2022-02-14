/**
 * Created by Ability Edge#18273
 * - Elior Yousefi, and Eitan Kravets
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

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
public class GreenLanternPipeline extends OpenCvPipeline
{
    public static double lowValuesTSEH = 80;
    public static double lowValuesTSES = 43;
    public static double lowValuesTSEV = 14;

    public static double highValuesTSEH = 111;
    public static double highValuesTSES = 173;
    public static double highValuesTSEV = 92;

    public static double lowValuesDUCKH = 21;
    public static double lowValuesDUCKS = 105;
    public static double lowValuesDUCKV = 80;

    public static double highValuesDUCKH = 86;
    public static double highValuesDUCKS = 216;
    public static double highValuesDUCKV = 167;

    // creating a mast with the same resolution of the webcam for the place to display the detected team shipping element
    Mat mask = new Mat(1280,720,0);//
    Mat inputHSV = new Mat(1280,720,0);

    public static boolean DEBUG = true;

    public static double threshold_percentage = 0.01; // 1% of the element color

    // the list of locations that can be
    public enum Location{
        Left,
        Center,
        Right,
        Not_Found
    }

    public Location location;

    // color for the rectangles to show on the screen
    Scalar colorBarcodeRectGREEN = new Scalar(0, 255, 0);
    Scalar colorBarcodeRectRED = new Scalar(255, 0, 0);

    public Telemetry telemetry;

    public static boolean TSE = false;

    boolean barcodeLeft, barcodeCenter, barcodeRight;

    Rect LEFT_SEC, CENTER_SEC, RIGHT_SEC;

    @Override
    public Mat processFrame(Mat input) {

        // creating 3 rectangles(sections) for checking the colors inside them.
        LEFT_SEC = new Rect(
                new Point(0,720),//mask.cols()/7, mask.rows()/5 * 2
                new Point(200,720));//mask.cols()/7, mask.rows()/5 * 2

        CENTER_SEC = new Rect(
                new Point(900,720),
                new Point(456.666667,0));

        RIGHT_SEC = new Rect(
                new Point(1280,720 ),
                new Point(900,0 ));


//        // HSV low and high values for our team shipping element.
//        Scalar lowValuesTSE = new Scalar(lowValuesTSEH, lowValuesTSES, lowValuesTSEV);
//        Scalar highValuesTSE = new Scalar(highValuesTSEH, highValuesTSES, highValuesTSEV);

        Scalar lowValuesDUCK = new Scalar(lowValuesDUCKH, lowValuesDUCKS, lowValuesDUCKV);
        Scalar highValuesDUCK = new Scalar(highValuesDUCKH, highValuesDUCKS, highValuesDUCKV);

        Imgproc.cvtColor(input, inputHSV, Imgproc.COLOR_RGB2HSV);

        // turning all colors not between the low and high values to black and the rest white.
        Core.inRange(inputHSV, lowValuesDUCK, highValuesDUCK, mask);
        telemetry.addLine("Using: DUCKS/TSE");

        // taking sections from the mask to another mat
        Mat left = mask.submat(LEFT_SEC);
        Mat center = mask.submat(CENTER_SEC);
        Mat right = mask.submat(RIGHT_SEC);

        // calculating the average amount of color in the SECTION.
        double leftAvg = Core.sumElems(left).val[0] / LEFT_SEC.area() / 255;
        double centerAvg = Core.sumElems(center).val[0] / CENTER_SEC.area() / 255;
        double rightAvg = Core.sumElems(right).val[0] / RIGHT_SEC.area() / 255;

        // we release the mats for use.
        left.release();
        center.release();
        right.release();

        // if debugging so we display more values in telemetry.
        if(DEBUG){
            threshold_percentage = 0;

            telemetry.addData("Left raw value", Core.sumElems(left).val[0]);
            telemetry.addData("Center raw value", Core.sumElems(center).val[0]);
            telemetry.addData("Right raw value", Core.sumElems(right).val[0]);
            telemetry.addData("Left average percentage", Math.round(leftAvg * 100) + "%");
            telemetry.addData("Center average percentage", Math.round(centerAvg * 100) + "%");
            telemetry.addData("Right average percentage", Math.round(rightAvg * 100) + "%");
            telemetry.addData("Barcode Location", getLocation());
            telemetry.update();
        }

        // comparing the average color in the left/center/right rect to the threshold.
        barcodeLeft = leftAvg > threshold_percentage;
        barcodeCenter = centerAvg > threshold_percentage;
        barcodeRight = rightAvg > threshold_percentage;

        //checking which barcode is found.
        if(barcodeLeft && barcodeCenter && barcodeRight){
            // NOT FOUND
            //location = Location.Not_Found;
            location = Location.Left;
            //telemetry.addData("Barcode Location:","Unknown Barcode.");
        }else if(barcodeLeft){
            location = Location.Left;
            //telemetry.addData("Barcode Location:","LEFT Barcode.");
        }else if(barcodeCenter){
            location = Location.Center;
            //telemetry.addData("Barcode Location:","CENTER Barcode.");
        }else if(barcodeRight){
            location = Location.Right;
            //telemetry.addData("Barcode Location:","RIGHT Barcode.");
        }else{
            // NOT FOUND
            //location = Location.Not_Found;
            location = Location.Left;
            //telemetry.addData("Barcode Location:","Unknown Barcode.");
        }
        telemetry.update();

        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);

        switch (location)
        {
            case Left:
                // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
                Imgproc.rectangle(mask, LEFT_SEC, colorBarcodeRectGREEN);
                Imgproc.rectangle(mask, CENTER_SEC, colorBarcodeRectRED);
                Imgproc.rectangle(mask, RIGHT_SEC, colorBarcodeRectRED);
                break;
            case Center:
                // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
                Imgproc.rectangle(mask, LEFT_SEC, colorBarcodeRectRED);
                Imgproc.rectangle(mask, CENTER_SEC, colorBarcodeRectGREEN);
                Imgproc.rectangle(mask, RIGHT_SEC, colorBarcodeRectRED);
                break;
            case Right:
                // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
                Imgproc.rectangle(mask, LEFT_SEC, colorBarcodeRectRED);
                Imgproc.rectangle(mask, CENTER_SEC, colorBarcodeRectRED);
                Imgproc.rectangle(mask, RIGHT_SEC, colorBarcodeRectGREEN);
                break;
            default:
                // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
                Imgproc.rectangle(mask, LEFT_SEC, colorBarcodeRectRED);
                Imgproc.rectangle(mask, CENTER_SEC, colorBarcodeRectRED);
                Imgproc.rectangle(mask, RIGHT_SEC, colorBarcodeRectRED);
                break;
        }

        return mask;
    }

    // getting location of the team shipping element.
    public Location getLocation() {
        Location location;

        if(barcodeLeft && barcodeCenter && barcodeRight){
            // NOT FOUND
            //location = Location.Not_Found;
            location = Location.Left;
            //telemetry.addData("Barcode Location:","Unknown Barcode.");
        }else if(barcodeLeft){
            location = Location.Left;
            //telemetry.addData("Barcode Location:","LEFT Barcode.");
        }else if(barcodeCenter){
            location = Location.Center;
            //telemetry.addData("Barcode Location:","CENTER Barcode.");
        }else if(barcodeRight){
            location = Location.Right;
            //telemetry.addData("Barcode Location:","RIGHT Barcode.");
        }else{
            // NOT FOUND
            //location = Location.Not_Found;
            location = Location.Left;
            //telemetry.addData("Barcode Location:","Unknown Barcode.");
        }

        return location;
    }

    public boolean isTSE() {
        return TSE;
    }

    public void setTSE(boolean TSE) {
        this.TSE = TSE;
    }

    public void setTelementry(Telemetry telemetry){ this.telemetry = telemetry; }
}