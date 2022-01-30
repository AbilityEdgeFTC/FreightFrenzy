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
    public static double timeTo = .5;
    Elevator elevator;
    dip dip;

    public enum ElevatorState
    {
        ZERO,
        MIN,
        MID,
        MAX
    }

    public static ElevatorState elevatorSate = ElevatorState.ZERO;

    public ElevatorThreadAuto(HardwareMap hw) throws InterruptedException {
        elevator = new Elevator(hw);
        dip = new dip(hw);

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
            NanoClock clock = NanoClock.system();

            while (!isInterrupted())
            {
                double startTime = clock.seconds();

                if(elevatorSate == ElevatorState.MIN)
                {
                    elevator.setHeight(Elevator.MIN_HEIGHT);
                }
                else if(elevatorSate == ElevatorState.MID)
                {
                    elevator.setHeight(Elevator.MID_HEIGHT);
                }
                else if(elevatorSate == ElevatorState.MAX)
                {
                    elevator.setHeight(Elevator.MAX_HEIGHT);
                }
                else if(elevatorSate == ElevatorState.ZERO)
                {
                    elevator.setHeight(Elevator.ZERO_HEIGHT);
                }

                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && elevatorSate == ElevatorState.MIN)
                {
                    elevator.update();
                }
                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && elevatorSate == ElevatorState.MID)
                {
                    elevator.update();
                }
                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && elevatorSate == ElevatorState.MAX)
                {
                    elevator.update();
                }while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && elevatorSate == ElevatorState.ZERO)
                {
                    elevator.update();
                }
            }
        }
        // an error occurred in the run loop.
        catch (Exception e) {}
    }
}
