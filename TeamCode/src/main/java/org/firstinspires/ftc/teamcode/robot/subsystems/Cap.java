/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Cap {

    CRServo spin;
    Servo upDown;
    Servo claw;
    public static double openingPosition = 0, closingPosition = 0;
    public static double holdUp = 0, holDown = 0, placeCap = 0;
    Gamepad gamepad;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public Cap(HardwareMap hardwareMap, Gamepad gamepad) {
        this.spin = hardwareMap.get(CRServo.class, "crSpin");
        this.upDown = hardwareMap.get(Servo.class, "sC");
        this.claw = hardwareMap.get(Servo.class, "sC");
        this.gamepad = gamepad;
    }

    public void update()
    {

    }
//
//    // move dip servo to intake position.
//    public void closeCover() {
//        sC.setPosition(closingPosition);
//    }
//
//    // move dip servo to releasing position.
//    public void openCover() {
//        sC.setPosition(openingPosition);
//    }
//
//    // get dip servo position
//    public double getPos() {
//        return sC.getPosition();
//    }
//
//    // move to custom position
//    public void moveTo(double position)
//    {
//        sC.setPosition(position);
//    }
//
}
