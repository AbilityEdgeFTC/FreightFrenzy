package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


/*
 * Hardware class for our elevator using FIRST'S set target position.
 */
@Config
public class ElevatorFirstPID {

    public static double HUB_LEVEL3 = 24.95;
    public static double HUB_LEVEL2 = 10;
    public static double HUB_LEVEL1 = HUB_LEVEL2;
    public static double AUTO_LEFT_LEVEL = 19;
    public static double DUCK_RED_LEVEL = 8.4;
    public static double MID = 6;
    public static double SHARED_HUB = 6.5;
    public static double ZERO_HEIGHT = 0;
    public static double TICKS_PER_REV = 145.1;
    public static double SPOOL_RADIUS = 0.75; // in
    public static double maxPower = 0.6;
    double power = 1;
    boolean usePID = true;
    public static int offset = 0;
    DcMotorEx motor;
    double target;
    Gamepad gamepad;
    cGamepad cGamepad;
    NanoClock clock;

    public enum ElevatorLevel {
        ZERO,
        SHARED_HUB,
        HUB_LEVEL1,
        HUB_LEVEL2,
        HUB_LEVEL3,
        AUTO_LEFT_LEVEL,
        DUCK_LEVEL,
        MID
    }

    public static ElevatorLevel elevatorLevel = ElevatorLevel.ZERO;

    public ElevatorFirstPID(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
        clock = NanoClock.system();
    }

    public ElevatorFirstPID(HardwareMap hardwareMap)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        cGamepad.update();

        if (usePID)
        {
            switch (elevatorLevel)
            {
                case ZERO:
                    target = 0;
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
                case MID:
                    target = MID;
                    break;
            }

            motor.setTargetPosition(inchesToEncoderTicks(target - ZERO_HEIGHT) + offset);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(gamepad.left_stick_y != 0 && !gamepad.left_stick_button)
            {
                motor.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
            }
            else if(gamepad.left_stick_y != 0 && gamepad.left_stick_button)
            {
                motor.setPower(Range.clip(-gamepad.left_stick_y, -1, 1));
            }
            else
            {
                motor.setPower(0);
            }
        }
    }

    public void updateAuto()
    {
        switch (elevatorLevel)
        {
            case ZERO:
                target = 0;
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
            case DUCK_LEVEL:
                target = DUCK_RED_LEVEL;
                break;
            case AUTO_LEFT_LEVEL:
                target = AUTO_LEFT_LEVEL;
                break;
            case MID:
                target = MID;
                break;
        }



        motor.setTargetPosition(inchesToEncoderTicks(target));
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
    ALOT OF GETTER AND SETTERS :)
     */
    public static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static int inchesToEncoderTicks(double inches) {
        return (int)Math.round((inches * TICKS_PER_REV) / (SPOOL_RADIUS * 2 * Math.PI));
    }

    public static int getOffset()
    {
        return offset;
    }

    public static void setOffset(int offset) {
        ElevatorFirstPID.offset = offset;
    }

    public int getPosition()
    {
        return motor.getCurrentPosition();
    }

    public double getTarget()
    {
        return target;
    }

    public int getTargetPosition()
    {
        return motor.getCurrentPosition();
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

    public double getPower() {
        return power;
    }

    public boolean getUsePID()
    {
        return usePID;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public static double getHubLevel3() {
        return HUB_LEVEL3;
    }

    public static void setHubLevel3(double hubLevel3) {
        HUB_LEVEL3 = hubLevel3;
    }

    public static void setHubLevel2(double hubLevel2) {
        HUB_LEVEL2 = hubLevel2;
    }

    public static void setHubLevel1(double hubLevel1) {
        HUB_LEVEL1 = hubLevel1;
    }

    public static void setAutoLeftLevel(double autoLeftLevel) {
        AUTO_LEFT_LEVEL = autoLeftLevel;
    }

    public static void setSharedHub(double sharedHub) {
        SHARED_HUB = sharedHub;
    }

    public static void setZeroHeight(double zeroHeight) {
        ZERO_HEIGHT = ZERO_HEIGHT;
    }

    public void resetElevator()
    {
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static double getZeroHeight() {
        return ZERO_HEIGHT;
    }

    public static double getMaxPower() {
        return maxPower;
    }

    public static void setMaxPower(double maxPower) {
        ElevatorFirstPID.maxPower = maxPower;
    }
}