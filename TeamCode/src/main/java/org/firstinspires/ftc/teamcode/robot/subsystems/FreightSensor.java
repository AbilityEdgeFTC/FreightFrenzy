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

    public FreightSensor(HardwareMap hardwareMap) {
        this.rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dsC");
    }


    public boolean hasFreight()
    {
        return rangeSensor.rawOptical() != emptyBox;
    }

}

