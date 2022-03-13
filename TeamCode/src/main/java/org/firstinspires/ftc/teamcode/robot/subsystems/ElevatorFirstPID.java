package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorFirstPID {

    public static double HUB_LEVEL3 = 22,HUB_LEVEL2 = 10.5,HUB_LEVEL1 = 10.5,AUTO_LEFT_LEVEL = 18,DUCK_RED_LEVEL = 10.5;
    public static double SHARED_HUB = 5.5;
    public static double ZERO_HEIGHT = 0;
    DcMotorEx motor;
    double target;
    public static double TICKS_PER_REV = 145.1;
    public static double SPOOL_RADIUS = 0.75; // in
    double power = 1;
    boolean usePID = true;
    public static double maxPower = 0.7;
    double startHeight = 0;
    public static boolean DEBUG = true;

    public enum ElevatorLevel {
        ZERO,
        SHARED_HUB,
        HUB_LEVEL1,
        HUB_LEVEL2,
        HUB_LEVEL3,
        AUTO_LEFT_LEVEL,
        DUCK_RED_LEVEL
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

        try {
            ZERO_HEIGHT = encoderTicksToInches(Integer.parseInt(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("ElevatorValue.txt"))));
        }catch (NumberFormatException e)
        {
            ZERO_HEIGHT = 0;
        }

        if(DEBUG)
        {
            ZERO_HEIGHT = 0;
        }
    }

    public ElevatorFirstPID(HardwareMap hardwareMap)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setDirection(DcMotor.Direction.REVERSE);

        try {
            ZERO_HEIGHT = encoderTicksToInches(Integer.parseInt(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("ElevatorValue.txt"))));
        }catch (NumberFormatException e)
        {
            ZERO_HEIGHT = 0;
        }

        if(DEBUG)
        {
            ZERO_HEIGHT = 0;
        }
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
                case DUCK_RED_LEVEL:
                    target = DUCK_RED_LEVEL;
                    break;
                case AUTO_LEFT_LEVEL:
                    target = AUTO_LEFT_LEVEL;
                    break;
            }

            motor.setTargetPosition(inchesToEncoderTicks(target - ZERO_HEIGHT));
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
        }
    }

    public void updateAuto()
    {
//        AUTO_LEFT_LEVEL = encoderTicksToInches(580);
//        DUCK_RED_LEVEL = encoderTicksToInches(220);

        if(usePID)
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
                case DUCK_RED_LEVEL:
                    target = DUCK_RED_LEVEL;
                    break;
                case AUTO_LEFT_LEVEL:
                    target = AUTO_LEFT_LEVEL;
                    break;
            }

            motor.setTargetPosition(inchesToEncoderTicks(target - ZERO_HEIGHT));
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    public double getHeight()
    {
        return encoderTicksToInches(motor.getCurrentPosition());
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
}