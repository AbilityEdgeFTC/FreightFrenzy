/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ElevatorThread extends Thread{

    public static boolean eliorPlaying = true;
    public static double timeTo = 1;
    Elevator elevator;
    Gamepad gamepad1;
    Telemetry telemetry;

    public enum ElevatorState
    {
        ZERO,
        MIN,
        MID,
        MAX
    }

    public static ElevatorState elevatorSate = ElevatorState.ZERO;

    public ElevatorThread(HardwareMap hw, Telemetry telemetry, Gamepad gamepad1) {
        elevator = new Elevator(hw);
        this.gamepad1 = gamepad1;
        eliorPlaying = false;
        elevatorSate = ElevatorState.ZERO;
        this.telemetry = telemetry;
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
                checkLevel();

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

//                if(gamepad1.right_bumper && (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down))
//                {
//                    elevator.motor.setPower(powerElevator);
//                    elevator.offset = elevator.motor.getCurrentPosition();
//                }
//                else if(gamepad1.left_bumper && (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down))
//                {
//                    elevator.motor.setPower(-powerElevator);
//                    elevator.offset = elevator.motor.getCurrentPosition();
//                }

                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && elevatorSate == ElevatorState.MIN)
                {
                    elevator.update();
                    checkLevel();
                }
                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && elevatorSate == ElevatorState.MID)
                {
                    elevator.update();
                    checkLevel();
                }
                while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && elevatorSate == ElevatorState.MAX)
                {
                    elevator.update();
                    checkLevel();
                }while (!isInterrupted() && (clock.seconds() - startTime) < timeTo && elevatorSate == ElevatorState.ZERO)
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
                elevatorSate = ElevatorState.MIN;
            } else if (gamepad1.b) {
                elevatorSate = ElevatorState.MID;
            } else if (gamepad1.y) {
                elevatorSate = ElevatorState.MAX;
            } else if (gamepad1.x) {
                elevatorSate = ElevatorState.ZERO;
            }
        }
        else
        {
            if (gamepad1.y) {
                elevatorSate = ElevatorState.MIN;
            } else if (gamepad1.b) {
                elevatorSate = ElevatorState.MID;
            } else if (gamepad1.x) {
                elevatorSate = ElevatorState.MAX;
            } else if (gamepad1.a) {
                elevatorSate = ElevatorState.ZERO;
            }
        }

        telemetry.addData("Power: ", elevator.getPower());
    }

}
