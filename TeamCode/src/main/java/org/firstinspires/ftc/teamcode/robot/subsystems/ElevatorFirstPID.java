package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorFirstPID {

    public static double HUB_LEVEL3 = 25,HUB_LEVEL2 = 19,HUB_LEVEL1 = 17;
    public static double SHARED_HUB = 4.5;
    public static double ZERO_HEIGHT = 0;
    DcMotorEx motor;
    double target;
    public static double TICKS_PER_REV = 145.1;
    public static double SPOOL_RADIUS = 0.75; // in
    public static double power = 1;
    public static boolean usePID = true;

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

    public ElevatorFirstPID(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setDirection(DcMotor.Direction.REVERSE);
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
    }

    public void update() {
        cGamepad.update();

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

            motor.setTargetPosition(inchesToEncoderTicks(target - ZERO_HEIGHT));
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(Range.clip(-gamepad.left_stick_y, -power, power));
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

    public static void setElevatorLevel(ElevatorLevel elevatorLevel) {
        ElevatorFirstPID.elevatorLevel = elevatorLevel;
    }

    public static double getPower() {
        return power;
    }

    public boolean getUsePID()
    {
        return usePID;
    }

    public static void setPower(double power) {
        ElevatorFirstPID.power = power;
    }
}