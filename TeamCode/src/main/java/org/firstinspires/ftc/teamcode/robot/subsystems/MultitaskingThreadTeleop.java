/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MultitaskingThreadTeleop extends Thread {

    intake intake;
    Gamepad gamepad1, gamepad2;
    dip dip;
    public static double powerIntake = 1;
    ElevatorThread elevator;
    //Elevator elevator;
    cGamepad cGamepad1, cGamepad2;
    boolean frontIntake = false, backIntake = false, activeOpMode;

    public MultitaskingThreadTeleop(HardwareMap hw, Gamepad gamepad1, Gamepad gamepad2, boolean activeOpMode) throws InterruptedException {
        intake = new intake(hw);
        dip = new dip(hw);
        this.activeOpMode = activeOpMode;
        elevator = new ElevatorThread(hw, gamepad2, activeOpMode);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run() {
        try {
            dip.getFreight();

            while (!isInterrupted() && activeOpMode) {
                cGamepad1.update();
                cGamepad2.update();

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
                    intake.powerIntake(-powerIntake);
                }
                else if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0)
                {
                    intake.stop();
                }

                if(gamepad2.dpad_up || (elevator.elevatorSate == ElevatorThread.ElevatorState.MIN) || (elevator.elevatorSate == ElevatorThread.ElevatorState.MID) || (elevator.elevatorSate == ElevatorThread.ElevatorState.MAX) && !(cGamepad2.rightBumperOnce() || cGamepad2.leftBumperOnce()))
                {
                    dip.releaseFreightPos();
                }
                else if(gamepad2.dpad_down || elevator.elevatorSate == ElevatorThread.ElevatorState.ZERO && !(cGamepad2.rightBumperOnce() || cGamepad2.leftBumperOnce()))
                {
                    dip.getFreight();
                }

                if(cGamepad2.rightBumperOnce() || cGamepad2.leftBumperOnce())
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