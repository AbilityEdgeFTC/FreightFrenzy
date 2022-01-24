/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ElevatorThread extends Thread{

    public static boolean moveToMin = false, moveToMid = false, moveToMax = false, moveToZero = false;
    public static double timeTo = 3;
    Elevator elevator;
    Gamepad gamepad1;

    public ElevatorThread(HardwareMap hw, Gamepad gamepad1)
    {
        elevator = new Elevator(hw);
        this.gamepad1 = gamepad1;
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run()
    {
        try
        {
            moveToMin = false;
            moveToMid = false;
            moveToMax = false;
            moveToZero = false;

            NanoClock clock = NanoClock.system();

            while (!isInterrupted())
            {
                double startTime = clock.seconds();
                checkLevel();

                if(moveToMin)
                {
                    elevator.setHeight(Elevator.MIN_HEIGHT);
                }
                else if(moveToMid)
                {
                    elevator.setHeight(Elevator.MID_HEIGHT);
                }
                else if(moveToMax)
                {
                    elevator.setHeight(Elevator.MAX_HEIGHT);
                }
                else if(moveToZero)
                {
                    elevator.setHeight(Elevator.ZERO_HEIGHT);
                }

                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && moveToMin)
                {
                    elevator.update();
                    checkLevel();
                }
                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && moveToMid)
                {
                    elevator.update();
                    checkLevel();
                }
                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && moveToMax)
                {
                    elevator.update();
                    checkLevel();
                }while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && moveToZero)
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
        if(gamepad1.a)
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
        else if(gamepad1.y)
        {
            moveToMax = true;
            moveToMin = false;
            moveToMid = false;
            moveToZero = false;
        }
        else if(gamepad1.x)
        {
            moveToZero = true;
            moveToMin = false;
            moveToMid = false;
            moveToMax = false;
        }
    }
}
