/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MultitaskingThreadTeleop extends Thread {

    public static boolean reverseElevatorSpinner;
    intake intake;
    Gamepad gamepad1;
    DcMotor mI,mE,mS;
    dip dip;
    public static double powerIntake = 1, powerSpinner = .2;
    private boolean toggleIntakeForward = false, toggleIntakeBackwards = false;

    public MultitaskingThreadTeleop(HardwareMap hw, Gamepad gamepad1) throws InterruptedException {
        mE = hw.get(DcMotor.class, "mE");
        mS = hw.get(DcMotor.class, "mS");
        if(reverseElevatorSpinner)
        {
            mS.setDirection(DcMotor.Direction.REVERSE);
        }
        else {
            mS.setDirection(DcMotor.Direction.FORWARD);
        }
        mS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = new intake(hw);
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

                mE.setPower(gamepad1.right_trigger);
                mE.setPower(-gamepad1.left_trigger);

                if (gamepad1.right_bumper)
                {
                    toggleIntakeForward = true;
                    toggleIntakeBackwards = false;
                }
                else if (gamepad1.left_bumper) {
                    toggleIntakeBackwards = true;
                    toggleIntakeForward = false;
                }

                if(gamepad1.b)
                {
                    mS.setPower(powerSpinner);
                }
                else if(gamepad1.x)
                {
                    mS.setPower(-powerSpinner);
                }
                else
                {
                    mS.setPower(0);
                }

                if(toggleIntakeForward)
                {
                    intake.powerIntake(powerIntake);
                }
                else if(toggleIntakeBackwards)
                {
                    intake.powerIntake(-powerIntake);
                }
                else
                {
                    intake.stop();
                    toggleIntakeBackwards = false;
                    toggleIntakeForward = false;
                }


                if(gamepad1.y)
                {
                    dip.releaseFreightPos();
                }
                else if(gamepad1.a)
                {
                    dip.getFreight();
                }
            }
        }
        // an error occurred in the run loop.
        catch (Exception e) {
        }
    }

}