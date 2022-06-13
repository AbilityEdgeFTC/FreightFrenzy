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
import com.qualcomm.robotcore.util.Range;

// 0,1 all the way down(the hand)
// .4 all the way up

@Config
public class Cap {

    CRServo spin;
    Servo upDown;
    Servo hook;
    public static double increment = 0.03, maxPower = .1;
    double position = 0;
    Gamepad gamepad;
    cGamepad cGamepad;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public Cap(HardwareMap hardwareMap, Gamepad gamepad) {
        this.spin = hardwareMap.get(CRServo.class, "sSpin");
        this.upDown = hardwareMap.get(Servo.class, "sUpDown");
        this.hook = hardwareMap.get(Servo.class, "sHook");
        this.upDown.setDirection(Servo.Direction.REVERSE);
        this.hook.setDirection(Servo.Direction.REVERSE);
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
    }

    public void update()
    {
        cGamepad.update();

        spin.setPower(Range.clip(gamepad.right_stick_x, -maxPower, maxPower));

        if(cGamepad.rightBumperOnce())
        {
            position += increment;
        }
        else if(cGamepad.leftBumperOnce())
        {
            position -= increment;
        }

        upDown.setPosition(position);
        hook.setPosition(position);


    }

}
