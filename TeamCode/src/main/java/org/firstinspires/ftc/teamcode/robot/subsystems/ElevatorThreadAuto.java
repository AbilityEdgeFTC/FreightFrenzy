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
public class ElevatorThreadAuto extends Thread{

    public static boolean eliorPlaying = false;
    public static double timeTo = 1;
    Elevator elevator;


    public enum ElevatorState
    {
        ZERO,
        MIN,
        MID,
        MAX,
        RELEASE
    }

    public static ElevatorState elevatorState = ElevatorState.ZERO;

    public ElevatorThreadAuto(HardwareMap hw) throws InterruptedException {
        elevator = new Elevator(hw);
        eliorPlaying = false;
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run()
    {
        try
        {
            NanoClock clock = NanoClock.system();

            while (!isInterrupted())
            {
                double startTime = clock.seconds();

                if(elevatorState == ElevatorState.MIN || elevatorState == ElevatorState.RELEASE)
                {
                    elevator.setHeight(Elevator.MIN_HEIGHT);
                }
                else if(elevatorState == ElevatorState.MID || elevatorState == ElevatorState.RELEASE)
                {
                    elevator.setHeight(Elevator.MID_HEIGHT);
                }
                else if(elevatorState == ElevatorState.MAX || elevatorState == ElevatorState.RELEASE)
                {
                    elevator.setHeight(Elevator.MAX_HEIGHT);
                }
                else if(elevatorState == ElevatorState.ZERO)
                {
                    elevator.setHeight(Elevator.ZERO_HEIGHT);
                }

                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && (elevatorState == ElevatorState.MIN || elevatorState == ElevatorState.RELEASE))
                {
                    elevator.update();
                }
                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && (elevatorState == ElevatorState.MID || elevatorState == ElevatorState.RELEASE))
                {
                    elevator.update();
                }
                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && (elevatorState == ElevatorState.MAX || elevatorState == ElevatorState.RELEASE))
                {
                    elevator.update();
                }while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && elevatorState == ElevatorState.ZERO)
                {
                    elevator.update();
                }
            }
        }
        // an error occurred in the run loop.
        catch (Exception e) {}
    }
}
