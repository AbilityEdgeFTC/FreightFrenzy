package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorSpinner {

    public static double MAX_ANGLE = 60;
    public static double MIN_ANGLE = -60;
    public static double ZERO_ANGLE = 0;
    DcMotorEx motor;
    public static PIDCoefficients coefficients = new PIDCoefficients(0,0,0);
    BasicPID PID;
    AngleController controller;
    double target;
    public static double TICKS_PER_REV = 384.5;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double power = 0.1;
    public static boolean usePID = true;

    public enum SpinnerState
    {
        ZERO,
        MIN,
        MAX
    }

    public static SpinnerState spinnerState = SpinnerState.ZERO;

    Gamepad gamepad;
    cGamepad cGamepad;

    public ElevatorSpinner(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PID = new BasicPID(new com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients(coefficients.kP, coefficients.kI, coefficients.kD));
        controller = new AngleController(PID);
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
                spinnerState = SpinnerState.MAX;
                target = inchesToEncoderTicks(MAX_ANGLE);
            }
            else if(gamepad.b)
            {
                spinnerState = SpinnerState.ZERO;
                target = inchesToEncoderTicks(ZERO_ANGLE);

            }
            else if(gamepad.a)
            {
                spinnerState = SpinnerState.MIN;
                target = inchesToEncoderTicks(MIN_ANGLE);
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

    }

    public static double encoderTicksToInches(int ticks) {
        return GEAR_RATIO * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static double inchesToEncoderTicks(double inches) {
        return (inches * TICKS_PER_REV) / (GEAR_RATIO * 2 * Math.PI);
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