/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class    FreightSensor {

    AnalogSensor freightSensor;
    public static double threshold = 0;
    Telemetry telemetry;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public FreightSensor(HardwareMap hardwareMap) {
        this.freightSensor = hardwareMap.get(AnalogSensor.class, "freightSensor");
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public FreightSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.freightSensor = hardwareMap.get(AnalogSensor.class, "freightSensor");
        this.telemetry = telemetry;
    }

    // spin dip servo to intake positions, and holding servo to hold position.
    public boolean freightOn()
    {
        if(freightSensor.readRawVoltage() > threshold)
        {
            return true;
        }

        return false;
    }

    // display position of servo's.
    public void displayTelemetry(){
        telemetry.addData("Sensor value", freightSensor.readRawVoltage());
        telemetry.update();
    }

}
