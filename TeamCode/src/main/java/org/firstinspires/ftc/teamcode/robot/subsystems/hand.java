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
public class hand {

    public static Servo sT;
    Telemetry telemetry;
    public static double down = 0., mid = 0.5, up = 1;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public hand(HardwareMap hardwareMap) {
        this.sT = hardwareMap.get(Servo.class, "sT");
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public hand(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sT = hardwareMap.get(Servo.class, "sT");
        this.telemetry = telemetry;
    }

    // spin dip servo to intake positions, and holding servo to hold position.
    public static void moveHand1() throws InterruptedException {
        sT.setPosition(up);
    }

    public static void moveHand2() throws InterruptedException {
        sT.setPosition(mid);
    }

    public static void moveHand3() throws InterruptedException {
        sT.setPosition(down);
    }

    public static void moveHand(double position) throws InterruptedException {
        sT.setPosition(position);
    }

    // display position of servo's.
    public void displayTelemetry(){
        telemetry.addLine("Servo hand at: " + sT.getPosition());
        telemetry.update();
    }

}
