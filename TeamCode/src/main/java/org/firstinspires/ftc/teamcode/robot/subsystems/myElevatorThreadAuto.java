/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class myElevatorThreadAuto extends Thread{

    myElevator elevator;

    public enum ElevatorSate
    {
        ZERO,
        MIN,
        MID,
        MAX,
        HOLD
    }

    public ElevatorSate elevatorSate = ElevatorSate.ZERO;

    public myElevatorThreadAuto(HardwareMap hw) throws InterruptedException {
        elevator = new myElevator(hw);
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
        switch (elevatorSate)
        {
            case ZERO:
                elevator.goToZero();
                break;
            case MIN:
                elevator.goToMin();
                break;
            case MID:
                elevator.goToMid();
                break;
            case MAX:
                elevator.goToMax();
                break;
            case HOLD:
                elevator.stop();
                break;
        }
    }
}
