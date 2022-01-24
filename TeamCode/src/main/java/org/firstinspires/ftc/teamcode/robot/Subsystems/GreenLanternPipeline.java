package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class GreenLanternPipeline extends OpenCvPipeline
{

    // HSV low and high values for our team shipping element.
    Scalar lowValuesTSE = new Scalar(86, 102, 11);
    Scalar highValuesTSE = new Scalar(108, 203, 102);

    Scalar lowValuesDUCK = new Scalar(86, 102, 11);
    Scalar highValuesDUCK = new Scalar(108, 203, 102);

    // creating a mast with the same resolution of the webcam for the place to display the detected team shipping element
    Mat mask = new Mat(1280,720,0);//

    // creating 3 rectangles(sections) for checking the colors inside them.
    final Rect LEFT_SEC = new Rect(
            new Point(426.666667,720),//mask.cols()/7, mask.rows()/5 * 2
            new Point(0,0));//mask.cols()/7 * 2, mask.rows()/5 * 4)


    final Rect CENTER_SEC = new Rect(
            new Point(853.333334,720),
            new Point(426.666667,0));

    final Rect RIGHT_SEC = new Rect(
            new Point(1280,720 ),
            new Point(853.333334,0 ));

    public boolean DEBUG;

    double threshold_percentage = 0.01; // 1% of the element color

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

    public boolean TSE = true;

    boolean barcodeLeft, barcodeCenter, barcodeRight;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);

        // turning all colors not between the low and high values to black and the rest white.
        if(TSE){
            Core.inRange(mask, lowValuesTSE, highValuesTSE, mask);
        }else if(!TSE){
            Core.inRange(mask, lowValuesDUCK, highValuesDUCK, mask);
        }else{
            Core.inRange(mask, lowValuesTSE, highValuesTSE, mask);
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
            telemetry.update();
        }

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

        // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
        Imgproc.rectangle(mask, LEFT_SEC, colorBarcodeRect);
        Imgproc.rectangle(mask, CENTER_SEC, colorBarcodeRect);
        Imgproc.rectangle(mask, RIGHT_SEC, colorBarcodeRect);

        return mask;
    }

    // getting location of the team shipping element.
    public Location getLocation() {
        Location location;

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