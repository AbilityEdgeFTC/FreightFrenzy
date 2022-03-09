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
public class MenuPipeline extends OpenCvPipeline
{
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

    public static double threshold = 10000;

    boolean barcodeLeft, barcodeCenter, barcodeRight;
    Scalar colorBarcodeRect = new Scalar(0, 255, 0);

    // creating a mast with the same resolution of the webcam for the place to display the detected team shipping element
    Mat mask = new Mat(1280,720,0);//
    Mat inputYCbCr = new Mat(1280,720,0);

    public static boolean DEBUG = true;

    public static double[] max ={255,255,255};
    public static double[] min ={0,0,0};

    public Telemetry telemetry;

    @Override
    public Mat processFrame(Mat input) {
        Scalar lowValues = new Scalar(min[0], min[1], min[2]);
        Scalar highValues = new Scalar(max[0], max[1], max[2]);

        // turning all colors not between the low and high values to black and the rest white.
        //Core.inRange(inputYCbCr, lowValues, highValues, mask);

        // taking sections from the mask to another mat
        /*Mat left = mask.submat(LEFT_SEC);
        Mat center = mask.submat(CENTER_SEC);
        Mat right = mask.submat(RIGHT_SEC);

        barcodeLeft = Core.countNonZero(left) > threshold;
        barcodeCenter = Core.countNonZero(center) > threshold;
        barcodeRight = Core.countNonZero(right) > threshold;

        // we release the mats for use.
        left.release();
        center.release();
        right.release();

        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);

        // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
        Imgproc.rectangle(mask, LEFT_SEC, colorBarcodeRect);
        Imgproc.rectangle(mask, CENTER_SEC, colorBarcodeRect);
        Imgproc.rectangle(mask, RIGHT_SEC, colorBarcodeRect);

        telemetry.update();*/

        return mask;
    }

    public void setTelementry(Telemetry telemetry){ this.telemetry = telemetry; }

    public void setMax(double[] max) {
        MenuPipeline.max = max;
    }

    public void setMin(double[] min) {
        MenuPipeline.min = min;
    }
}