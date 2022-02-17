package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class Elevator {

    public static double MAX_HEIGHT = 10;
    public static double MID_HEIGHT = 5;
    public static double MIN_HEIGHT = 2.5;
    public static double ZERO_HEIGHT = 0;
    DcMotorEx motor;
    public static PIDCoefficients coefficients = new PIDCoefficients(0.011,0.5,0);
    BasicPID controller;
    double target;
    public static double TICKS_PER_REV = 384.5;
    public static double SPOOL_RADIUS = 0.75; // in
    public static boolean usePID = true;
    public static double power = 0.2;

    public enum ElevatorState
    {
        ZERO,
        MIN,
        MID,
        MAX
    }

    public static ElevatorState elevatorSate = ElevatorState.ZERO;

    Gamepad gamepad;
    cGamepad cGamepad;

    public Elevator(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new BasicPID(new com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients(coefficients.kP, coefficients.kI, coefficients.kD));
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
    }

    public void update()
    {
        if(cGamepad.dpadUpOnce() || cGamepad.dpadDownOnce())
        {
            usePID = !usePID;
        }

        if(usePID)
        {
            if(gamepad.y)
            {
                elevatorSate = ElevatorState.MAX;
                target = inchesToEncoderTicks(MAX_HEIGHT);
            }
            else if(gamepad.b)
            {
                elevatorSate = ElevatorState.MID;
                target = inchesToEncoderTicks(MID_HEIGHT);

            }
            else if(gamepad.a)
            {
                elevatorSate = ElevatorState.MIN;
                target = inchesToEncoderTicks(MIN_HEIGHT);
            }
            else if(gamepad.x)
            {
                elevatorSate = ElevatorState.ZERO;
                target = inchesToEncoderTicks(ZERO_HEIGHT);
            }
        }
        else
        {
            if(gamepad.right_bumper)
            {
                motor.setPower(power);
            }
            else if(gamepad.left_bumper)
            {
                motor.setPower(-power);
            }
            else
            {
                motor.setPower(0);
            }
        }

        controller.calculate(target, motor.getCurrentPosition());

    }

    public static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static double inchesToEncoderTicks(double inches) {
        return (inches * TICKS_PER_REV) / (SPOOL_RADIUS * 2 * Math.PI);
    }

    public int getPosition()
    {
        return motor.getCurrentPosition();
    }

    public double getTarget()
    {
        return target;
    }



}