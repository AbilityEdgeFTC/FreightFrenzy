package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorSpinnerRegular {

    public static double RIGHT_ANGLE = 60;
    public static double LEFT_ANGLE = -60;
    public static double ZERO_ANGLE = 0;
    public static double power = 0.7;
    public static boolean usePID = true;
    double target = 0;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    DcMotorEx motor;

    public enum SpinnerState
    {
        LEFT,
        ZERO,
        RIGHT
    }

    public static SpinnerState spinnerState = SpinnerState.ZERO;

    Gamepad gamepad;
    cGamepad cGamepad;

    public ElevatorSpinnerRegular(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
                    target = Math.toRadians(LEFT_ANGLE);
                    break;
                case RIGHT:
                    target = Math.toRadians(RIGHT_ANGLE);
                    break;
            }

            motor.setTargetPosition((int)radiansToEncoderTicks(Angle.normDelta(encoderTicksToRadians(motor.getCurrentPosition()))));
            //motor.setTargetPosition((int)(radiansToEncoderTicks(Angle.normDelta(target))));
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            motor.setPower(gamepad.right_stick_x);

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

    public void setTarget(double newTarget)
    {
        target = newTarget;
    }

    public void setUsePID(boolean usePID)
    {
        this.usePID = usePID;
    }

    public static SpinnerState getSpinnerState() {
        return spinnerState;
    }

    public static void setSpinnerState(SpinnerState spinnerState) {
        ElevatorSpinnerRegular.spinnerState = spinnerState;
    }
}