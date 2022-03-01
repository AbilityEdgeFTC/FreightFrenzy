package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ElevatorSpinnerCOMPLEX_UNSTABLE {

    public static double MAX_ANGLE = 60;
    public static double MIN_ANGLE = -60;
    public static double ZERO_ANGLE = 0;
    public static double power = 0.2;
    public static boolean usePID = true;
    double target = 0;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    DcMotorEx motor;
    public static PIDCoefficients coefficients = new PIDCoefficients(2.5,0,0);
    public static PIDCoefficients coefficientsVAS = new PIDCoefficients(0,0,0);
    PIDFController controller = new PIDFController(coefficients,coefficientsVAS.kP, coefficientsVAS.kI, coefficientsVAS.kD);
    public static double MAX_ACCEL = 2.5;
    public static double MAX_VEL = 5;

    public enum SpinnerState
    {
        LEFT,
        ZERO,
        RIGHT
    }

    public static SpinnerState spinnerState = SpinnerState.ZERO;

    Gamepad gamepad;
    cGamepad cGamepad;

    public ElevatorSpinnerCOMPLEX_UNSTABLE(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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

            controller.setTargetPosition(radiansToEncoderTicks(target));
            controller.setTargetVelocity(MAX_VEL);
            controller.setTargetAcceleration(MAX_ACCEL);

            motor.setPower(controller.update(radiansToEncoderTicks(Angle.normDelta(encoderTicksToRadians(motor.getCurrentPosition()))), motor.getVelocity()));
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
        ElevatorSpinnerCOMPLEX_UNSTABLE.spinnerState = spinnerState;
    }
}