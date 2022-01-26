/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class myElevatorThread extends Thread{

    public static boolean moveToMin = false, moveToMid = false, moveToMax = false, moveToZero = false, eliorPlaying = false;
    public static double timeTo = 2;
    myElevator elevator;
    Gamepad gamepad1;
    dip dip;

    public myElevatorThread(HardwareMap hw, Gamepad gamepad1) throws InterruptedException {
        elevator = new myElevator(hw);
        dip = new dip(hw);
        this.gamepad1 = gamepad1;
        moveToMin = false;
        moveToMid = false;
        moveToMax = false;
        moveToZero = false;
        eliorPlaying = false;
        dip.getFreight();
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run()
    {
        try
        {

            while (!isInterrupted())
            {
                checkLevel();

                if(moveToMin)
                {
                    dip.releaseFreightPos();
                    elevator.setHeight(Elevator.MIN_HEIGHT);
                }
                else if(moveToMid)
                {
                    dip.releaseFreightPos();
                    elevator.setHeight(Elevator.MID_HEIGHT);
                }
                else if(moveToMax)
                {
                    dip.releaseFreightPos();
                    elevator.setHeight(Elevator.MAX_HEIGHT);
                }
                else if(moveToZero)
                {
                    dip.getFreight();
                    elevator.setHeight(Elevator.ZERO_HEIGHT);
                }

                while (!isInterrupted() && moveToMin)
                {
                    elevator.update();
                    checkLevel();
                }
                while (!isInterrupted() && moveToMid)
                {
                    elevator.update();
                    checkLevel();
                }
                while (!isInterrupted() && moveToMax)
                {
                    elevator.update();
                    checkLevel();
                }while (!isInterrupted() && moveToZero)
                {
                    elevator.update();
                    checkLevel();
                }
            }
        }
        // an error occurred in the run loop.
        catch (Exception e) {}
    }

    void checkLevel() throws InterruptedException {

        if(eliorPlaying) {
            if (gamepad1.a) {
                moveToMin = true;
                moveToMid = false;
                moveToMax = false;
                moveToZero = false;
            } else if (gamepad1.b) {
                moveToMid = true;
                moveToMin = false;
                moveToMax = false;
                moveToZero = false;
            } else if (gamepad1.y) {
                moveToMax = true;
                moveToMin = false;
                moveToMid = false;
                moveToZero = false;
            } else if (gamepad1.x) {
                moveToZero = true;
                moveToMin = false;
                moveToMid = false;
                moveToMax = false;
            }
        }
        else
        {
            if(gamepad1.y)
            {
                moveToMin = true;
                moveToMid = false;
                moveToMax = false;
                moveToZero = false;
            }
            else if(gamepad1.b)
            {
                moveToMid = true;
                moveToMin = false;
                moveToMax = false;
                moveToZero = false;
            }
            else if(gamepad1.x)
            {
                moveToMax = true;
                moveToMin = false;
                moveToMid = false;
                moveToZero = false;
            }
            else if(gamepad1.a)
            {
                moveToZero = true;
                moveToMin = false;
                moveToMid = false;
                moveToMax = false;
            }
        }

    }
}
