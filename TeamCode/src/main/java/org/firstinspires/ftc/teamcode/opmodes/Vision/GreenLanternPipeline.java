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
public class GreenLanternPipeline extends OpenCvPipeline
{
    public static double lowValuesY = 33;
    public static double lowValuesB = 165;
    public static double lowValuesR = 73;

    public static double highValuesY = 155;
    public static double highValuesB = 235;
    public static double highValuesR = 117;

    // creating a mast with the same resolution of the webcam for the place to display the detected team shipping element
    Mat mask = new Mat(1280,720,0);//
    Mat YCrCb = new Mat(1280,720,0);

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
    Scalar lowValues, highValues;

    public Telemetry telemetry;

    public static boolean redSide = true;

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


//        if(redSide)
//        {
            lowValues = new Scalar(lowValuesY, lowValuesB, lowValuesR);
            highValues = new Scalar(highValuesY, highValuesB, highValuesR);
//        }
//        else
//        {
//            lowValues = new Scalar(lowYBlue, lowCrBlue, lowCbBlue);
//            highValues = new Scalar(highYBlue, highCrBlue, highCbBlue);
//        }

        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_BGR2YCrCb);

        // turning all colors not between the low and high values to black and the rest white.
        Core.inRange(YCrCb, lowValues, highValues, mask);
        if(redSide)
        {
            telemetry.addData("Alliance color", "Red");
        }
        else
        {
            telemetry.addData("Alliance color", "Blue");
        }

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
        }
        telemetry.update();

        // comparing the average color in the left/center/right rect to the threshold.
        barcodeLeft = leftAvg > threshold_percentage;
        barcodeCenter = centerAvg > threshold_percentage;
        barcodeRight = rightAvg > threshold_percentage;

        //checking which barcode is found.
        if(barcodeLeft && barcodeCenter && barcodeRight){
            // NOT FOUND
            location = Location.Not_Found;
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
            location = Location.Not_Found;
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
            location = Location.values()[(int) Math.random() * (2 - 0 + 1) + 0];
            telemetry.addData("Barcode Location:","Not found Barcode.");
        }else if(barcodeLeft){
            location = Location.Left;
        }else if(barcodeCenter){
            location = Location.Center;
        }else if(barcodeRight){
            location = Location.Right;
        }else{
            // NOT FOUND
            location = Location.values()[(int) Math.random() * (2 - 0 + 1) + 0];
            telemetry.addData("Barcode Location:","Not found Barcode.");
        }

        return location;
    }

    public boolean isRedSide() {
        return redSide;
    }

    public void setRedSide(boolean redSide) {
        this.redSide = redSide;
    }

    public void setTelementry(Telemetry telemetry){ this.telemetry = telemetry; }
}