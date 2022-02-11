package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class myElevator {
    PIDController controller;

    public static double MAX_HEIGHT = 20;
    public static double MID_HEIGHT = 13;
    public static double MIN_HEIGHT = 9;
    public static double ZERO_HEIGHT = 0;

    public enum ElevatorState
    {
        ZERO,
        MIN,
        MID,
        MAX
    }

    public static ElevatorState elevatorSate = ElevatorState.ZERO;

    Gamepad gamepad;

    public myElevator(HardwareMap hardwareMap, Gamepad gamepad)
    {
        controller = new PIDController(hardwareMap);
        this.gamepad = gamepad;
    }

    public void update(ElapsedTime time)
    {
        if(gamepad.y)
        {
            elevatorSate = ElevatorState.MAX;
        }
        else if(gamepad.b)
        {
            elevatorSate = ElevatorState.MID;
        }
        else if(gamepad.a)
        {
            elevatorSate = ElevatorState.MIN;
        }
        else if(gamepad.x)
        {
            elevatorSate = ElevatorState.ZERO;
        }

        switch (elevatorSate)
        {
            case ZERO:
                controller.updatePosition(ZERO_HEIGHT, time);
                break;
            case MIN:
                controller.updatePosition(MIN_HEIGHT, time);
                break;
            case MID:
                controller.updatePosition(MID_HEIGHT, time);
                break;
            case MAX:
                controller.updatePosition(MAX_HEIGHT, time);
                break;
        }
    }



}