package org.firstinspires.ftc.teamcode.opmodes.Vision;

public class VisionObject {


    private static double highValueTSE;
    private static double lowValueTSE;
    private static String type;

    public VisionObject(double highValuesTSE,double lowValuesTSE,String type)
    {
        this.lowValueTSE = lowValuesTSE;
        this.highValueTSE= highValuesTSE;
        this.type = type;
    }

    public static String getType() {
        return type;
    }

    public static double getHighValueTSE() {
        return highValueTSE;
    }

    public static double getLowValueTSE() {
        return lowValueTSE;
    }

    public static void setHighValueTSE(double highValueTSE) {
        VisionObject.highValueTSE = highValueTSE;
    }

    public static void setLowValueTSE(double lowValueTSE) {
        VisionObject.lowValueTSE = lowValueTSE;
    }

}
