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
public class dip {

    Servo sD, sH;
    public static double intakePosition = .42, releasingPosition = .25, holdingPosition = 1, pushingPosition = .4;
    Telemetry telemetry;
    public static double time = 100;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public dip(HardwareMap hardwareMap) {
        this.sD = hardwareMap.get(Servo.class, "sE");
        this.sH = hardwareMap.get(Servo.class, "sH");;
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public dip(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sD = hardwareMap.get(Servo.class, "sE");
        this.sH = hardwareMap.get(Servo.class, "sH");;
        this.telemetry = telemetry;
    }

    // spin dip servo to intake positions, and holding servo to hold position.
    public void getFreight() throws InterruptedException {
        sD.setPosition(intakePosition);
        sH.setPosition(holdingPosition);
        Thread.sleep((long)time);
    }
    // spin dip servo to releasing positions, and holding servo to hold position.
    public void releaseFreightPos() throws InterruptedException {
        sD.setPosition(releasingPosition);
        sH.setPosition(holdingPosition);
        Thread.sleep((long)time);
    }

    // spin holding servo to push position.
    public void releaseFreight() throws InterruptedException {
        sH.setPosition(pushingPosition);
        Thread.sleep((long)time);
    }

    // display position of servo's.
    public void displayTelemetry(){
        telemetry.addLine("Servo Dip at: " + sD.getPosition());
        telemetry.addLine("Servo Hold at: " + sH.getPosition());
        telemetry.update();
    }

}
