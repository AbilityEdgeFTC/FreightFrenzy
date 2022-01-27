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
public class transportation {

    Servo sR, sL;
    public static double rightPos1 = 0, leftPos1 = 0;
    public static double rightPos2 = 0, leftPos2 = 0;
    public static double rightPos3 = 0, leftPos3 = 0;
    public static double rightPos4 = 0, leftPos4 = 0;
    Telemetry telemetry;
    public static double time = 500;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public transportation(HardwareMap hardwareMap) {
        this.sR = hardwareMap.get(Servo.class, "sRT");
        this.sL = hardwareMap.get(Servo.class, "sLT");;
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public transportation(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sR = hardwareMap.get(Servo.class, "sRT");
        this.sL = hardwareMap.get(Servo.class, "sLT");;
        this.telemetry = telemetry;
    }

    // spin dip servo to intake positions, and holding servo to hold position.
    public void moveToPos1() throws InterruptedException {
        sR.setPosition(rightPos1);
        sL.setPosition(leftPos1);
        Thread.sleep((long)time);
    }

    public void moveToPos2() throws InterruptedException {
        sR.setPosition(rightPos2);
        sL.setPosition(leftPos2);
        Thread.sleep((long)time);
    }
    public void moveToPos3() throws InterruptedException {
        sR.setPosition(rightPos3);
        sL.setPosition(leftPos3);
        Thread.sleep((long)time);
    }
    public void moveToPos4() throws InterruptedException {
        sR.setPosition(rightPos4);
        sL.setPosition(leftPos4);
        Thread.sleep((long)time);
    }
    // display position of servo's.
    public void displayTelemetry(){
        telemetry.addLine("Servo Right at: " + sR.getPosition());
        telemetry.addLine("Servo Left at: " + sL.getPosition());
        telemetry.update();
    }

}
