package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ElevatorCOMPLEX_UNSTABLE {

    public static double HUB_LEVEL1 = 22,HUB_LEVEL2 = 19,HUB_LEVEL3 = 17;
    public static double SHARED_HUB = 4.5;
    public static double ZERO_HEIGHT = 0;
    DcMotorEx motor;
    public static PIDCoefficients coefficients = new PIDCoefficients(.05,0,0);
    public static PIDCoefficients coefficientsVAS = new PIDCoefficients(0,0,0);
    double target;
    public static double TICKS_PER_REV = 384.5;
    public static double SPOOL_RADIUS = 0.75; // in
    public static double power = 0.5;
    public static boolean usePID = true;
    public static double MAX_ACCEL = 5;
    public static double MAX_VEL = 5;
    PIDFController controller = new PIDFController(coefficients,coefficientsVAS.kP, coefficientsVAS.kI, coefficientsVAS.kD);
    NanoClock clock;

    public enum ElevatorLevel {
        ZERO,
        SHARED_HUB,
        HUB_LEVEL1,
        HUB_LEVEL2,
        HUB_LEVEL3
    }

    public static ElevatorLevel elevatorLevel = ElevatorLevel.ZERO;

    Gamepad gamepad;
    cGamepad cGamepad;

    public ElevatorCOMPLEX_UNSTABLE(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotor.Direction.REVERSE);
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);

    }

    public void update()
    {
        if (usePID)
        {
            switch (elevatorLevel)
            {
                case ZERO:
                    target = ZERO_HEIGHT;
                    break;
                case SHARED_HUB:
                    target = SHARED_HUB;
                    break;
                case HUB_LEVEL1:
                    target = HUB_LEVEL1;
                    break;
                case HUB_LEVEL2:
                    target = HUB_LEVEL2;
                    break;
                case HUB_LEVEL3:
                    target = HUB_LEVEL3;
                    break;

            }
            controller.setTargetPosition(inchesToEncoderTicks(target));
            controller.setTargetVelocity(MAX_VEL);
            controller.setTargetAcceleration(MAX_ACCEL);

            motor.setPower(controller.update(motor.getCurrentPosition(), motor.getVelocity()));
        }
        else
        {
            motor.setPower(-gamepad.left_stick_y);
        }
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

    public double getTarget()
    {
        return target;
    }

    public static boolean getUsePID()
    {
        return usePID;
    }

    public void setTarget(double newTarget)
    {
        target = newTarget;
    }

    public void setUsePID(boolean usePID)
    {
        this.usePID = usePID;
    }

    public static ElevatorLevel getElevatorLevel() {
        return elevatorLevel;
    }

    public static void setSpinnerState(ElevatorLevel elevatorLevel) {
        ElevatorCOMPLEX_UNSTABLE.elevatorLevel = elevatorLevel;
    }

}