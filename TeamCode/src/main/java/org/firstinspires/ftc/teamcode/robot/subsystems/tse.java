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
public class tse {

    Servo sT;
    public static double pos1 = 0, pos2 = 0, pos3 = 0;
    Telemetry telemetry;
    public static double time = 100;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public tse(HardwareMap hardwareMap) {
        this.sT = hardwareMap.get(Servo.class, "sT");
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public tse(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sT = hardwareMap.get(Servo.class, "sT");
        this.telemetry = telemetry;
    }

    // spin dip servo to intake positions, and holding servo to hold position.
    public void goToPos1() throws InterruptedException {
        sT.setPosition(pos1);
        Thread.sleep((long)time);
    }
    // spin dip servo to releasing positions, and holding servo to hold position.
    public void goToPos2() throws InterruptedException {
        sT.setPosition(pos2);
        Thread.sleep((long)time);
    }
    // spin dip servo to releasing positions, and holding servo to hold position.
    public void goToPos3() throws InterruptedException {
        sT.setPosition(pos3);
        Thread.sleep((long)time);
    }

    // display position of servo's.
    public void displayTelemetry(){
        telemetry.addLine("Servo Dip at: " + sT.getPosition());
        telemetry.update();
    }

}
