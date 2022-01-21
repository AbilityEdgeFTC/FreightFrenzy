package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class myElevator extends Thread{
    public static final double TICKS_PER_REV = 384.5;
    public static double SPOOL_RADIUS = 0.75; // in

    public static double intakePosition = .46, dippingPosition = .3;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double MAX_HEIGHT = 15.5; // TODO set value in inches
    public static double MID_HEIGHT = 9; // TODO set value in inches
    public static double MIN_HEIGHT = 4; // TODO set value in inches
    public static double ZERO_HEIGHT = 0; // TODO set value in inches

    double power;
    int offset;

    DcMotorEx motor;
    PIDCoefficients pid = new PIDCoefficients(kP,kI,kD);
    PIDFController controller;

    Gamepad gamepad;
    Telemetry telemetry;
    dip dip;
    Servo sD;

    public static boolean moveToMin = false, moveToMid = false, moveToMax = false, moveToZero = false;

    public myElevator(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad)
    {
        motor = hardwareMap.get(DcMotorEx.class, "mE");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // if necessary, reverse the motor so "up" is positive
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDFController(pid,kF);
        sD = hardwareMap.get(Servo.class, "sE");
        dip = new dip(sD, intakePosition, dippingPosition);
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    @Override
    public void run()
    {
        try
        {
            while (!interrupted())
            {
                checkLevel();

                if(moveToMin)
                {
                    goToMin();
                }
                else if(moveToMid)
                {
                    goToMid();
                }
                else if(moveToMax)
                {
                    goToMax();
                }
                else if(moveToZero) {
                    goToZero();
                    dip.getFreight();
                }

                telemetry.addData("VELOCITY: ", getVelocity());
                telemetry.addData("TARGET VELOCITY: ", getTargetVelocity());
                telemetry.addData("ERROR: ", controller.getLastError());
                telemetry.update();
            }
        }
        catch (Exception e) {}
    }

    void checkLevel()
    {
        if(gamepad.a)
        {
            moveToMin = true;
            moveToMid = false;
            moveToMax = false;
            moveToZero = false;
        }
        else if(gamepad.b)
        {
            moveToMid = true;
            moveToMin = false;
            moveToMax = false;
            moveToZero = false;
        }
        else if(gamepad.y)
        {
            moveToMax = true;
            moveToMin = false;
            moveToMid = false;
            moveToZero = false;
        }
        else if(gamepad.x)
        {
            moveToZero = true;
            moveToMin = false;
            moveToMid = false;
            moveToMax = false;
        }
    }

    public void goToMax()
    {
        goToPoistion(MAX_HEIGHT);
    }

    public void goToMid()
    {
        goToPoistion(MID_HEIGHT);
    }

    public void goToMin()
    {
        goToPoistion(MIN_HEIGHT);

    }

    public void goToZero()
    {
        goToPoistion(ZERO_HEIGHT);
    }

    void goToPoistion(double height)
    {
        controller.setTargetPosition(inchesToEncoderTicks(height));
        power = controller.update(motor.getCurrentPosition()-offset, motor.getVelocity());

        motor.setPower(power);
    }

    public double getVelocity()
    {
        return motor.getVelocity();
    }

    public double getTargetVelocity()
    {
        return controller.getTargetVelocity();
    }

    private static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    private static double inchesToEncoderTicks(double inches) {
        return (TICKS_PER_REV * inches) / (2 * SPOOL_RADIUS * Math.PI);
    }
}
