package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.opencv.core.Mat;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorSpinner {

    public static double MAX_ANGLE = 60;
    public static double MIN_ANGLE = -60;
    public static double ZERO_ANGLE = 0;
    public static double power = 0.2;
    public static boolean usePID = true;
    public static boolean stopAndReset = true;
    BasicPID PID;
    AngleController controller;
    public static double kP = 2.7;
    public static double kI = 0;
    public static double kD = 0;
    public static double target = 0;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    DcMotor motor;

    public static int spinnerLevel = 0;
    public enum SpinnerState
    {
        LEFT,
        ZERO,
        RIGHT
    }

    public static SpinnerState spinnerState = SpinnerState.ZERO;

    Gamepad gamepad;
    cGamepad cGamepad;

    public ElevatorSpinner(HardwareMap hardwareMap, Gamepad gamepad)
    {
        PID = new BasicPID(new PIDCoefficients(kP, kI, kD));
        controller = new AngleController(PID);
        motor = hardwareMap.get(DcMotorEx.class, "mS");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(stopAndReset)
        {
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
    }

    public void update()
    {
        cGamepad.update();

        if(usePID)
        {
            switch (spinnerState)
            {
                case ZERO:
                    target = Math.toRadians(ZERO_ANGLE);
                    break;
                case LEFT:
                    target = Math.toRadians(MIN_ANGLE);
                    break;
                case RIGHT:
                    target = Math.toRadians(MAX_ANGLE);
                    break;
            }
            motor.setPower(controller.calculate(target, encoderTicksToRadians(motor.getCurrentPosition())));
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

    public static double encoderTicksToRadians(int ticks) {
        return Math.toRadians((ticks * 360) / TICKS_PER_REV);
    }

    public static double radiansToEncoderTicks(double radians) {
        return TICKS_PER_REV / (radians * 2 * Math.PI);
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