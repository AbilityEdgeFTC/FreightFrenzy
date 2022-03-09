package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class SpinnerFirstPID {

    public static int RIGHT_ANGLE = 215;
    public static int LEFT_ANGLE = -215;
    public static int ZERO_ANGLE = 315;
    public static double power = 1;
    public static boolean usePID = true;
    int target = 0;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    DcMotorEx motor;

    public enum SpinnerState
    {
        LEFT,
        ZERO_RED,
        ZERO_BLUE,
        ZERO_DO_NOT_USE,
        RIGHT
    }

    public static SpinnerState spinnerState = SpinnerState.ZERO_DO_NOT_USE;

    Gamepad gamepad;
    cGamepad cGamepad;

    public SpinnerFirstPID(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
    }

    public SpinnerFirstPID(HardwareMap hardwareMap)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void update()
    {
        cGamepad.update();

        if(usePID)
        {
            switch (spinnerState)
            {
                case ZERO_DO_NOT_USE:
                    target = 0;
                    break;
                case LEFT:
                    target = LEFT_ANGLE;
                    break;
                case RIGHT:
                    target = RIGHT_ANGLE;
                    break;
                case ZERO_RED:
                case ZERO_BLUE:
                    target = ZERO_ANGLE;
                    break;
            }

            motor.setTargetPosition(target - ZERO_ANGLE);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(Range.clip(gamepad.left_stick_x, -power/2, power/2));

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

    public double getTargetRadians()
    {
        return encoderTicksToRadians(target);
    }

    public int getTarget()
    {
        return target;
    }

    public void setTarget(int newTarget)
    {
        target = newTarget;
    }

    public void setUsePID(boolean usePID)
    {
        this.usePID = usePID;
    }

    public boolean getUsePID()
    {
        return usePID;
    }

    public static SpinnerState getSpinnerState() {
        return spinnerState;
    }

    public static void setSpinnerState(SpinnerState spinnerState) {
        SpinnerFirstPID.spinnerState = spinnerState;
    }
}