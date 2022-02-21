package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class Elevator {

    public static double MAX_HEIGHT = 13;
    public static double MID_HEIGHT = 10;
    public static double MIN_HEIGHT = 6;
    public static double ZERO_HEIGHT = 0;
    DcMotorEx motor;
    public static PIDCoefficients coefficients = new PIDCoefficients(.05,0,0);
    BasicPID controller;
    public static double target;
    public static double TICKS_PER_REV = 384.5;
    public static double SPOOL_RADIUS = 0.75; // in
    public static double power = 0.5;
    public static boolean stopAndReset = true;
    public static boolean usePID = true;

    Gamepad gamepad;
    cGamepad cGamepad;

    public Elevator(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotor.Direction.REVERSE);
        if(stopAndReset)
        {
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        controller = new BasicPID(new com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients(coefficients.kP, coefficients.kI, coefficients.kD));
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
    }

    public void update()
    {
        if (usePID)
        {
            motor.setPower(controller.calculate(inchesToEncoderTicks(target), motor.getCurrentPosition()));
        }
        else
        {
            if (gamepad.right_trigger != 0) {
                motor.setPower(gamepad.right_trigger);
            } else if (gamepad.left_trigger != 0) {
                motor.setPower(-gamepad.left_trigger);
            } else {
                motor.setPower(0);
            }

        }
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