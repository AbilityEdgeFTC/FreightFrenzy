/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ElevatorThreadAuto extends Thread{

    public static double timeTo = 1;
    public static Elevator elevator;

    public enum ElevatorState
    {
        ZERO,
        MIN,
        MID,
        MAX
    }

    public static ElevatorState elevatorState = ElevatorState.ZERO;

    public ElevatorThreadAuto(HardwareMap hw) {
        elevator = new Elevator(hw);
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run()
    {
        try
        {
            NanoClock clock = NanoClock.system();
            elevatorState = ElevatorState.ZERO;

            while (!isInterrupted())
            {
                double startTime = clock.seconds();

                if(elevatorState == ElevatorState.MIN)
                {
                    elevator.setHeight(Elevator.MIN_HEIGHT);
                }
                else if(elevatorState == ElevatorState.MID)
                {
                    elevator.setHeight(Elevator.MID_HEIGHT);
                }
                else if(elevatorState == ElevatorState.MAX)
                {
                    elevator.setHeight(Elevator.MAX_HEIGHT);
                }
                else if(elevatorState == ElevatorState.ZERO)
                {
                    elevator.setHeight(Elevator.ZERO_HEIGHT);
                }

                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && (elevatorState == ElevatorState.MIN))
                {
                    elevator.update();
                }
                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && (elevatorState == ElevatorState.MID))
                {
                    elevator.update();
                }
                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && (elevatorState == ElevatorState.MAX))
                {
                    elevator.update();
                }while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && elevatorState == ElevatorState.ZERO)
                {
                    elevator.update();
                }
            }

            Thread.currentThread().interrupt();
        }catch (Exception e){}
    }

    public static void setElevatorState(ElevatorState elevatorState) {
        ElevatorThreadAuto.elevatorState = elevatorState;
    }
}
