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

// 0,1 all the way down(the hand)
// .4 all the way up

@Config
public class Cap {

    CRServo spin;
    Servo upDown;
    Servo hook;
    public static double hookDown = 0, hookUp = 0.4;
    public static double getCap = 0, placeCap = 0.4, increment = 0.05;
    public static double devideSpinnerPower = 4;
    double position = placeCap;
    Gamepad gamepad;
    cGamepad cGamepad;

    boolean toggleHookAngleFix = true;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public Cap(HardwareMap hardwareMap, Gamepad gamepad) {
        this.spin = hardwareMap.get(CRServo.class, "crSpin");
        this.upDown = hardwareMap.get(Servo.class, "sUpDown");
        this.hook = hardwareMap.get(Servo.class, "sHook");
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
    }

    public void update()
    {
        spin.setPower(gamepad.right_stick_x / devideSpinnerPower);

        if(cGamepad.YOnce())
        {
            position = placeCap;
            toggleHookAngleFix = true;
        }
        else if(cGamepad.BOnce())
        {
            position = getCap;
            toggleHookAngleFix = false;
            hook.setPosition(hookDown);
        }
        else if(cGamepad.dpadUpOnce())
        {
            position += increment;
        }
        else if(cGamepad.dpadDownOnce())
        {
            position -= increment;
        }

        upDown.setPosition(position);

        if(toggleHookAngleFix)
        {
            hook.setPosition(position);
        }


    }

}
