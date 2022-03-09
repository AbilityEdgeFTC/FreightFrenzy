package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorSpinnerLibraryPID {

    double RIGHT_ANGLE = 0, RIGHT_ANGLE_SHARED = 0, LEFT_ANGLE = 0, LEFT_ANGLE_SHARED = 0, ZERO_ANGLE = 0;
    public static double power = 0.2;
    boolean usePID = true;
    public static double kP = 6;
    public static double kI = 0;
    public static double kD = 0;
    double target = 0;
    public static double maxPower = 0.4;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    DcMotorEx motor;
    BasicPID PID = new BasicPID(new PIDCoefficients(kP, kI, kD));
    AngleController controller = new AngleController(PID);

    public enum SpinnerState
    {
        LEFT,
        ZERO_RED,
        ZERO_BLUE,
        ZERO_DO_NOT_USE,
        SHARED_RED,
        SHARED_BLUE,
        RIGHT
    }

    public static SpinnerState spinnerState = SpinnerState.ZERO_DO_NOT_USE;

    Gamepad gamepad;
    cGamepad cGamepad;

    public ElevatorSpinnerLibraryPID(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
        ZERO_ANGLE = encoderTicksToRadians(315);
        LEFT_ANGLE_SHARED = encoderTicksToRadians(-150);
        LEFT_ANGLE = encoderTicksToRadians(-203);
        RIGHT_ANGLE_SHARED = encoderTicksToRadians(150);
        RIGHT_ANGLE = encoderTicksToRadians(203);
    }

    public ElevatorSpinnerLibraryPID(HardwareMap hardwareMap)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
                    target = ZERO_ANGLE;
                    break;
                case ZERO_BLUE:
                    target = ZERO_ANGLE;
                    break;
                case SHARED_RED:
                    target = RIGHT_ANGLE_SHARED;
                    break;
                case SHARED_BLUE:
                    target = LEFT_ANGLE_SHARED;
                    break;
            }

            motor.setPower(controller.calculate(target - ZERO_ANGLE, encoderTicksToRadians(motor.getCurrentPosition())));
        }
        else
        {
            motor.setPower(Range.clip(gamepad.right_stick_x, -maxPower, maxPower));

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

    public boolean getUsePID()
    {
        return usePID;
    }

    public void setUsePID(boolean usePID)
    {
        this.usePID = usePID;
    }

    public static SpinnerState getSpinnerState() {
        return spinnerState;
    }

    public static void setSpinnerState(SpinnerState spinnerState) {
        ElevatorSpinnerLibraryPID.spinnerState = spinnerState;
    }

}