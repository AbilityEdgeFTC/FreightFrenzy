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
 * Hardware class for our elevator spinner using custom aggressive pid.
 */
@Config
public class SpinnerPID {

    public static double power = 0.1;
    boolean usePID = true;
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;
    int pos = 0;
    public static double maxPower = 0.3;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    boolean slowMove = false;
    DcMotorEx motor;
    BasicPID PID = new BasicPID(new PIDCoefficients(kP, kI, kD));
    AngleController controller = new AngleController(PID);

    Gamepad gamepad2, gamepad1;
    cGamepad cGamepad2;

    public SpinnerPID(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.cGamepad2 = new cGamepad(gamepad2);
    }

    public void update()
    {
        cGamepad2.update();

        if(usePID)
        {
            motor.setPower(controller.calculate(encoderTicksToRadians(pos), encoderTicksToRadians(motor.getCurrentPosition())));
        }
        else
        {
            cGamepad2.update();

            if(slowMove && gamepad1.right_stick_x != 0 && !gamepad1.right_stick_button)
            {
                motor.setPower(Range.clip(gamepad1.right_stick_x, -maxPower, maxPower));
            }
            else if(gamepad2.right_stick_x != 0 && !gamepad2.right_stick_button)
            {
                motor.setPower(Range.clip(gamepad2.right_stick_x, -maxPower, maxPower));
            }
            else if(gamepad2.right_stick_x == 0 && gamepad1.right_stick_x == 0)
            {
                motor.setPower(0);
            }
            else if(slowMove && gamepad1.right_stick_x != 0 && gamepad1.right_stick_button)
            {
                motor.setPower(Range.clip(gamepad1.right_stick_x, -1, 1));
            }
            else if(gamepad2.right_stick_x != 0 && gamepad2.right_stick_button)
            {
                motor.setPower(Range.clip(gamepad2.right_stick_x, -1, 1));
            }
            else if(gamepad2.right_stick_x == 0 && gamepad1.right_stick_x == 0)
            {
                motor.setPower(0);
            }

            pos = getPosition();

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

    public boolean getUsePID()
    {
        return usePID;
    }

    public void setUsePID(boolean usePID)
    {
        this.usePID = usePID;
    }

    public boolean isSlowMove() {
        return slowMove;
    }

    public void setSlowMove(boolean slowMove) {
        this.slowMove = slowMove;
    }

    public static double getMaxPower() {
        return maxPower;
    }

    public static void setMaxPower(double maxPower) {
        SpinnerPID.maxPower = maxPower;
    }
}