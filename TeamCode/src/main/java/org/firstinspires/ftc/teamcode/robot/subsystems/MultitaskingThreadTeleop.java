/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MultitaskingThreadTeleop extends Thread {

    intake intake;
    Gamepad gamepad1, gamepad2;
    DcMotor mI;
    dip dip;
    public static double powerIntake = 1;
    myElevator elevator;
    //Elevator elevator;
    cGamepad cGamepad1;
    boolean frontIntake = false, backIntake = false;

    public MultitaskingThreadTeleop(HardwareMap hw, Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {
        intake = new intake(hw);
        dip = new dip(hw);
        dip.getFreight();
        elevator = new myElevator(hw);
        //elevator = new Elevator(hw);
        cGamepad1 = new cGamepad(gamepad1);
        this.gamepad2 = gamepad2;
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

                if (cGamepad1.rightBumperOnce())
                {
                    frontIntake = !frontIntake;
                    backIntake = false;
                }
                else if (cGamepad1.leftBumperOnce()) {
                    backIntake = !backIntake;
                    frontIntake = false;
                }


                if(frontIntake)
                {
                    intake.powerIntake(powerIntake);
                }
                else if(backIntake)
                {
                    intake.powerIntake(powerIntake);
                }
                else
                {
                    intake.stop();
                }

                if(gamepad2.dpad_up || elevator.moveToMax || elevator.moveToMid || elevator.moveToMin)
                {
                    dip.releaseFreightPos();
                }
                else if(gamepad2.dpad_down || elevator.moveToZero)
                {
                    dip.getFreight();
                }

                if(gamepad2.right_bumper || gamepad1.left_bumper)
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