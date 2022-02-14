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
    public static double intakePosition = 0.55, releasingPosition = .3, holdingPosition = 1, pushingPosition = .3;
    Telemetry telemetry;

    public enum HandState
    {
        HOLD,
        RELEASE
    }

    public static HandState handState = HandState.HOLD;

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
    }
    // spin dip servo to releasing positions, and holding servo to hold position.
    public void releaseFreightPos() throws InterruptedException {
        sD.setPosition(releasingPosition);
        sH.setPosition(holdingPosition);
    }

    // spin holding servo to push position.
    public void releaseFreight() throws InterruptedException {
        sH.setPosition(pushingPosition);
    }

    // spin dip servo to intake positions, and holding servo to hold position.
    public void getFreightAUTO() {
        sD.setPosition(intakePosition);
        sH.setPosition(holdingPosition);
    }
    // spin dip servo to releasing positions, and holding servo to hold position.
    public void releaseFreightPosAUTO() {
        sD.setPosition(releasingPosition);
        sH.setPosition(holdingPosition);
    }

    // spin holding servo to push position.
    public void releaseFreightAUTO() {
        sH.setPosition(pushingPosition);
    }

    // display position of servo's.
    public void displayTelemetry(){
        telemetry.addLine("Servo Dip at: " + sD.getPosition());
        telemetry.addLine("Servo Hold at: " + sH.getPosition());
        telemetry.update();
    }

}
