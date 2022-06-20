package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@Config
public class FreightSensor {

    ModernRoboticsI2cRangeSensor rangeSensor;
    public static double emptyBox = 0;
    public static boolean useSensor = true;

    public FreightSensor(HardwareMap hardwareMap) {
        this.rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dsC");
    }


    public boolean hasFreight()
    {
        if(useSensor)
        {
            return rangeSensor.rawOptical() != emptyBox;
        }
        else
        {
            return false;
        }
    }

    public static boolean isUseSensor() {
        return useSensor;
    }

    public static void setUseSensor(boolean useSensor) {
        FreightSensor.useSensor = useSensor;
    }
}

