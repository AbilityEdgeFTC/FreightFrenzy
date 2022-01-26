/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MultitaskingThreadTeleop extends Thread {

    intake intake;
    Gamepad gamepad1;
    DcMotor mI;
    dip dip;
    public static double powerIntake = 1;

    public MultitaskingThreadTeleop(HardwareMap hw, Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {
        mI = hw.get(DcMotor.class, "mI");
        mI.setDirection(DcMotor.Direction.REVERSE);
        intake = new intake(mI);
        dip = new dip(hw);
        dip.getFreight();
        this.gamepad1 = gamepad1;
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run() {
        try {
            while (!isInterrupted()) {
                if (gamepad1.left_trigger != 0)
                {
                    intake.powerIntake(-gamepad1.left_trigger);
                }
                else if (gamepad1.right_trigger != 0)
                {
                    intake.powerIntake(gamepad1.right_trigger);
                }
                else if (gamepad1.right_bumper)
                {
                    intake.powerIntake(powerIntake);
                }
                else if (gamepad1.left_bumper) {
                    intake.powerIntake(-powerIntake);
                }
                else
                {
                    intake.stop();
                }

                if(gamepad1.dpad_up)
                {
                    dip.releaseFreightPos();
                }
                else if(gamepad1.dpad_down)
                {
                    dip.getFreight();
                }

                if(gamepad1.right_bumper || gamepad1.left_bumper)
                {
                    dip.releaseFreight();
                }
            }
        }
        // an error occurred in the run loop.
        catch (Exception e) {
        }
    }

}