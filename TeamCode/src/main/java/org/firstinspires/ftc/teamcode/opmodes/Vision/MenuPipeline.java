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
//    public static int hMax = 255;
//    public static int sMax = 255;
//    public static int vMax = 255;
//
//    public static int hMin = 0;
//    public static int sMin = 0;
//    public static int vMin = 0;
//
//    public static int[] max ={hMax,sMax,vMax};
//    public static int[] min ={hMin,sMin,vMin};

    // creating a mast with the same resolution of the webcam for the place to display the detected team shipping element
    Mat mask = new Mat(1280,720,0);//
    Mat inputHSV = new Mat(1280,720,0);

    public static boolean DEBUG = true;

    public static double[] max ={255,255,255};
    public static double[] min ={0,0,0};
    public Telemetry telemetry;

    @Override
    public Mat processFrame(Mat input) {
        // HSV low and high values for our team shipping element.
        Scalar highValuesTSE = new Scalar(max[0], max[1], max[2]);
        Scalar lowValuesTSE = new Scalar(min[0], min[1], min[2]);

//        Scalar lowValuesDUCK = new Scalar(hMin, sMin, vMin);
//        Scalar highValuesDUCK = new Scalar(hMax, sMax, vMax);

        Imgproc.cvtColor(input, inputHSV, Imgproc.COLOR_RGB2HSV);

        // turning all colors not between the low and high values to black and the rest white.
//        if(TSE){
//            Core.inRange(inputHSV, lowValuesTSE, highValuesTSE, mask);
//            telemetry.addLine("Using: TSE");
//        }else if(!TSE){
//            Core.inRange(inputHSV, lowValuesDUCK, highValuesDUCK, mask);
//            telemetry.addLine("Using: DUCKS");
//        }else{
//            Core.inRange(inputHSV, lowValuesDUCK, highValuesDUCK, mask);
//            telemetry.addLine("Using: NONE");
//        }

        // taking sections from the mask to another mat
        //Mat all = mask.submat(ALL_SEC);

        // calculating the average amount of color in the SECTION.
        //double allAvg = Core.sumElems(all).val[0] / ALL_SEC.area()/255;
        // we release the mats for use.
        //all.release();
        // if debugging so we display more values in telemetry.

        Core.inRange(inputHSV, lowValuesTSE, highValuesTSE, mask);

        telemetry.update();

        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);

        // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
        //Imgproc.rectangle(mask, ALL_SEC, colorBarcodeRect);

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