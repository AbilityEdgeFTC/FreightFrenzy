package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ElevatorThread extends Thread{

    public static double MAX_HEIGHT = 15.5; // TODO set value in inches
    public static double MID_HEIGHT = 9; // TODO set value in inches
    public static double MIN_HEIGHT = 4; // TODO set value in inches
    public static double ZERO_HEIGHT = 0; // TODO set value in inches
    public static boolean moveToMin = false, moveToMid = false, moveToMax = false, moveToZero = false;
    public static double timeTo = 3;
    Elevator elevator;
    Gamepad gamepad1;

    public ElevatorThread(Telemetry telemetry, HardwareMap hw, Gamepad gamepad1)
    {
        telemetry.addData("Thread Called: ", this.getName());
        elevator = new Elevator(hw, MAX_HEIGHT, MID_HEIGHT, MIN_HEIGHT, ZERO_HEIGHT);
        this.gamepad1 = gamepad1;
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
                else if(moveToZero) {
                    elevator.setHeight(Elevator.ZERO_HEIGHT);
                }

                goToPoistions(clock.seconds(), startTime);

            }
        }
        // an error occurred in the run loop.
        catch (Exception e) {}
    }

    void checkLevel()
    {
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

    void goToPoistions(double seconds, double startTime)
    {
        while ((seconds - startTime) < timeTo && moveToMin)
        {
            elevator.update();
            checkLevel();
        }
        while((seconds - startTime) < timeTo && moveToMid)
        {
            elevator.update();
            checkLevel();
        }
        while((seconds - startTime) < timeTo && moveToMax)
        {
            elevator.update();
            checkLevel();
        }
        while((seconds - startTime) < timeTo && moveToZero)
        {
            elevator.update();
            checkLevel();
        }
    }
}
