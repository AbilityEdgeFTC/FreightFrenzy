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
public class ElevatorAuto {

    public static double MAX_HEIGHT = 13;
    public static double MID_HEIGHT = 10;
    public static double MIN_HEIGHT = 6;
    public static double ZERO_HEIGHT = 0;
    DcMotorEx motor;
    public static double TICKS_PER_REV = 384.5;
    public static double SPOOL_RADIUS = 0.75; // in
    public static boolean stopAndReset = true;

    public ElevatorAuto(HardwareMap hardwareMap)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setDirection(DcMotor.Direction.REVERSE);
        if(stopAndReset)
        {
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void goToPosition(double inches)
    {
        motor.setTargetPosition(inchesToEncoderTicks(inches));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static int inchesToEncoderTicks(double inches) {
        return (int)Math.round((inches * TICKS_PER_REV) / (SPOOL_RADIUS * 2 * Math.PI));
    }

    public int getPosition()
    {
        return motor.getCurrentPosition();
    }



}