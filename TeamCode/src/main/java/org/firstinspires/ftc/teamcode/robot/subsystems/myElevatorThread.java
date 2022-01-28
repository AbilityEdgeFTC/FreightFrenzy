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

    public static boolean eliorPlaying = false;
    myElevator elevator;
    Gamepad gamepad1;

    public myElevatorThread(HardwareMap hw, Gamepad gamepad1) throws InterruptedException {
        elevator = new myElevator(hw);
        this.gamepad1 = gamepad1;
        eliorPlaying = false;
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
                goToLevel();

                while(!elevator.controller.atSetPoint())
                {
                    elevator.update();
                    goToLevel();
                }

                elevator.stop();
            }
        }
        // an error occurred in the run loop.
        catch (Exception e) {}
    }

    void goToLevel() throws InterruptedException {
        if(eliorPlaying) {
            if (gamepad1.a)
            {
                elevator.goToMin();
            } else if (gamepad1.b)
            {
                elevator.goToMid();
            } else if (gamepad1.y)
            {
                elevator.goToMax();
            } else if (gamepad1.x)
            {
                elevator.goToZero();
            }
        }
        else
        {
            if (gamepad1.y)
            {
                elevator.goToMin();
            } else if (gamepad1.b)
            {
                elevator.goToMid();
            } else if (gamepad1.x)
            {
                elevator.goToMax();
            } else if (gamepad1.a)
            {
                elevator.goToZero();
            }
        }
    }
}
