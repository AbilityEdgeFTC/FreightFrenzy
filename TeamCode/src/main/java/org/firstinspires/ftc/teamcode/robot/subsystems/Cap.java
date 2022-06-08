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


@Config
public class Cap {

    CRServo spin;
    Servo upDown;
    Servo hook;
    public static double openingPosition = 0, closingPosition = 0;
    public static double holdUp = 0, holDown = 0, placeCap = 0;
    Gamepad gamepad;
    cGamepad cGamepad;

    boolean isToggle = false;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public Cap(HardwareMap hardwareMap, Gamepad gamepad) {
        this.spin = hardwareMap.get(CRServo.class, "crSpin");
        this.upDown = hardwareMap.get(Servo.class, "sC");
        this.hook = hardwareMap.get(Servo.class, "sC");
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
    }

    public void update()
    {
        spin.setPower(gamepad.left_stick_x);
        if (isToggle == false)
        {
            upDown.setPosition(gamepad.left_stick_y);
            hook.setPosition(gamepad.left_stick_y);
        }
        else
        {
            upDown.setPosition(gamepad.left_stick_y);
            hook.setPosition(gamepad.right_stick_y);
        }
        if (cGamepad.AOnce())
            isToggle = !isToggle;
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
