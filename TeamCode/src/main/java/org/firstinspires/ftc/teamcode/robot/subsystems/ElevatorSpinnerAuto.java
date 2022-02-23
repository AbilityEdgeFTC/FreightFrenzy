package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorSpinnerAuto {

    public static double MAX_ANGLE = 60;
    public static double MIN_ANGLE = -60;
    public static double ZERO_ANGLE = 0;
    public static boolean stopAndReset = true;
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

    public ElevatorSpinnerAuto(HardwareMap hardwareMap)
    {
        motor = hardwareMap.get(DcMotorEx.class, "mS");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(stopAndReset)
        {
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void goToPosition(double angle)
    {
        motor.setTargetPosition(radiansToEncoderTicks(Math.toRadians(angle)));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static double encoderTicksToRadians(int ticks) {
        return (Math.toRadians((ticks * 360) / TICKS_PER_REV));
    }

    public static int radiansToEncoderTicks(double radians) {
        return (int)(TICKS_PER_REV / (radians * 2 * Math.PI));
    }

    public int getPosition()
    {
        return motor.getCurrentPosition();
    }



}