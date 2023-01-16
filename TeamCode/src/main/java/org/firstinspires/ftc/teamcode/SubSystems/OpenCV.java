package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.SubSystems.OpenCV.imageType.Not_Found;

import android.location.Location;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


@Autonomous(name = "OpenCVTesting" , group = "autonomous")

public class OpenCV extends OpenCvPipeline {

    // HSV low and high values for our team shipping element.
    Scalar lowValuesTSE = new Scalar(86, 102, 11);
    Scalar highValuesTSE = new Scalar(108, 203, 102);

    Scalar lowValuesBulb = new Scalar(277, 87, 94);
    Scalar highValuesBulb = new Scalar(277, 83, 94);

    Scalar lowValuesLighting = new Scalar(277, 87, 94);
    Scalar highValuesLighting = new Scalar(277, 83, 94);

    Scalar lowValuesSolar = new Scalar(277, 87, 94);
    Scalar highValuesSolar = new Scalar(277, 83, 94);
    // creating a mast with the same resolution of the webcam for the place to display the detected team shipping element
    Mat mask = new Mat(1280, 720, 0);//

    public static boolean TSE = false;

    // creating 3 rectangles(sections) for checking the colors inside them.
    /*final Rect LEFT_SEC = new Rect(
            new Point(426.666667, 720),//mask.cols()/7, mask.rows()/5 * 2
            new Point(0, 0));//mask.cols()/7 * 2, mask.rows()/5 * 4)

     */


    final Rect LIGHTING_SEC = new Rect(
            new Point(853.333334, 720),
            new Point(426.666667, 0));

    final Rect BULB_SEC_ = new Rect(
            new Point(853.333334, 720),
            new Point(426.666667, 0));
    final Rect SOLAR_SEC = new Rect(
            new Point(853.333334, 720),
            new Point(426.666667, 0));

    /*final Rect RIGHT_SEC = new Rect(
            new Point(1280, 720),
            new Point(853.333334, 0));

     */

    boolean DEBUG;


    double threshold_percentage = 0.01; // 1% of the element color

    // the list of locations that can be
    public enum imageType {
        yellowBulb,
        greenLighting,
        purpleSolarPanel,
        Not_Found
    }

    private imageType imageColor;

    // color for the rectangles to show on the screen
    Scalar colorBarcodeRect = new Scalar(0, 255, 0);

    Telemetry telemetry;

    public boolean DetecedBulb = true;
    public boolean DetecedLighting = true;

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);

        // turning all colors not between the low and high values to black and the rest white.
        if (DetecedBulb) {
            Core.inRange(mask, lowValuesBulb, highValuesBulb, mask);
        } else if (!DetecedBulb) {
            Core.inRange(mask, lowValuesLighting, highValuesLighting, mask);
        } else if (!DetecedLighting && !DetecedBulb) {
            Core.inRange(mask, lowValuesSolar, highValuesSolar, mask);
        }

    /*    // taking sections from the mask to another mat
        // Mat left = mask.submat(LEFT_SEC);
        Mat center = mask.submat(CENTER_SEC);
        // Mat right = mask.submat(RIGHT_SEC);

        // calculating the average amount of color in the SECTION.
        // double leftAvg = Core.sumElems(left).val[0] / LEFT_SEC.area() / 255;
        double lightingAvg = Core.sumElems(center).val[0] / CENTER_SEC.area() / 255;
        // double rightAvg = Core.sumElems(right).val[0] / RIGHT_SEC.area() / 255;

        // we release the mats for use.
        // left.release();
        center.release();
        // right.release();

        // if debugging so we display more values in telemetry.
        if (DEBUG) {
            threshold_percentage = 0;

            // telemetry.addData("Left raw value", Core.sumElems(left).val[0]);
            telemetry.addData("Center raw value", Core.sumElems(center).val[0]);
            // telemetry.addData("Right raw value", Core.sumElems(right).val[0]);
            //  telemetry.addData("Left average percentage", Math.round(leftAvg * 100) + "%");
            // telemetry.addData("Center average percentage", Math.round(centerAvg * 100) + "%");
            // telemetry.addData("Right average percentage", Math.round(rightAvg * 100) + "%");
            // telemetry.addData("Barcode Location", getLocation());
            telemetry.update();
        }

        // comparing the average color in the left/center/right rect to the threshold.
        boolean bulb = leftAvg > threshold_percentage;
        boolean lighting = centerAvg > threshold_percentage;
        // boolean barcodeRight = rightAvg > threshold_percentage;

        //checking which barcode is found.
        if (lighting && barcodeCenter && barcodeRight) {
            // NOT FOUND
            location = imageType.Not_Found;
            //telemetry.addData("Barcode Location:","Unknown Barcode.");
        } else if (barcodeLeft) {
            location = imageType.;
            //telemetry.addData("Barcode Location:","LEFT Barcode.");
        } else if (barcodeCenter) {
            location = Location.Center;
            //telemetry.addData("Barcode Location:","CENTER Barcode.");
        } else if (barcodeRight) {
            location = Location.Right;
            //telemetry.addData("Barcode Location:","RIGHT Barcode.");
        } else {
            // NOT FOUND
            location = Not_Found;
            //telemetry.addData("Barcode Location:","Unknown Barcode.");
        }
        //telemetry.update();

        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);

        // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
        Imgproc.rectangle(mask, LEFT_SEC, colorBarcodeRect);
        Imgproc.rectangle(mask, CENTER_SEC, colorBarcodeRect);
        Imgproc.rectangle(mask, RIGHT_SEC, colorBarcodeRect);
        */
        return mask;
    }

    // getting location of the team shipping element.
    public imageType getLocation() {
        return imageColor;
    }

    public boolean isTSE() {
        return TSE;
    }

    public void setTSE(boolean TSE) {
        this.TSE = TSE;
    }


}