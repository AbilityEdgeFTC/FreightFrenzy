package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;

@Config
public class SensorColor {

    RevColorSensorV3 colorSensor;
    public static double gain = 2;
    float[] hsvValues = new float[3];
    NormalizedRGBA colors;

    public SensorColor(HardwareMap hardwareMap) {
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        colorSensor.setGain((float)gain);
    }

    public float[] getRGB()
    {
        colors = colorSensor.getNormalizedColors();
        float[] rgbValues ={colors.red, colors.blue, colors.green};
        return rgbValues;
    }

    public float[] getHSV()
    {
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues;
    }

    public float getAlpha()
    {
        colors = colorSensor.getNormalizedColors();
        return colors.alpha;
    }

    public boolean passedWearHouse()
    {
        return (hsvValues[2]>=0.05) && hsvValues[2]<=1;
    }

}

