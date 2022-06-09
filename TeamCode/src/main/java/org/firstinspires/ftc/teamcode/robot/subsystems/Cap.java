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

// 0 all the way down
// .4 all the way up

@Config
public class Cap {

    CRServo spin;
    Servo upDown;
    Servo hook;
    public static double hookDown = 0, hookUp = 0.4;
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
        spin.setPower(gamepad.left_stick_x);

        if(cGamepad.AOnce())
        {
            toggleHookAngleFix = !toggleHookAngleFix;
        }

        upDown.setPosition(gamepad.left_stick_y);

        if (!toggleHookAngleFix)
        {
            if(cGamepad.YOnce())
            {
                hook.setPosition(hookUp);
                toggleHookAngleFix = true;
            }
            else
            {
                if(cGamepad.BOnce())
                {
                    hook.setPosition(hookDown);
                }
            }
        }
        else
        {
            hook.setPosition(gamepad.right_stick_y);
        }
    }

}
