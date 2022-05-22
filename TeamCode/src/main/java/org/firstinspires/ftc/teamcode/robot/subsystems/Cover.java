/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Cover {

    Servo sC;
    public static double openingPosition = .5, closingPosition = .15;
    Telemetry telemetry;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public Cover(HardwareMap hardwareMap) {
        this.sC = hardwareMap.get(Servo.class, "sC");
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public Cover(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sC = hardwareMap.get(Servo.class, "sC");
        this.telemetry = telemetry;
    }

    // move dip servo to intake position.
    public void closeCover() {
        sC.setPosition(closingPosition);
    }

    // move dip servo to releasing position.
    public void openCover() {
        sC.setPosition(openingPosition);
    }

    // get dip servo position
    public double getPos() {
        return sC.getPosition();
    }

    // move to custom position
    public void moveTo(double position)
    {
        sC.setPosition(position);
    }

}
