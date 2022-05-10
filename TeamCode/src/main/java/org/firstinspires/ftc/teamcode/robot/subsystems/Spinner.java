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
public class Spinner {

    public static double power = 0.2;
    public static double maxPower = 0.4;
    boolean slowMove = false;
    DcMotorEx motor;

    Gamepad gamepad2, gamepad1;
    cGamepad cGamepad2;

    public Spinner(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.cGamepad2 = new cGamepad(gamepad2);
    }

    public void updateGamepad()
    {
        cGamepad2.update();

        if(slowMove && gamepad1.right_stick_x != 0 && !gamepad1.right_stick_button)
        {
            motor.setPower(Range.clip(gamepad1.right_stick_x, -maxPower, maxPower));
        }
        if(gamepad2.right_stick_x != 0 && !gamepad2.right_stick_button)
        {
            motor.setPower(Range.clip(gamepad2.right_stick_x, -maxPower, maxPower));
        }
        if(gamepad2.right_stick_x == 0 && gamepad1.right_stick_x == 0)
        {
            motor.setPower(0);
        }

        if(slowMove && gamepad1.right_stick_x != 0 && gamepad1.right_stick_button)
        {
            motor.setPower(Range.clip(gamepad1.right_stick_x, -maxPower/2, maxPower/2));
        }
        if(gamepad2.right_stick_x != 0 && gamepad2.right_stick_button)
        {
            motor.setPower(Range.clip(gamepad2.right_stick_x, -maxPower/2, maxPower/2));
        }
        if(gamepad2.right_stick_x == 0 && gamepad1.right_stick_x == 0)
        {
            motor.setPower(0);
        }

    }

    public int getPosition()
    {
        return motor.getCurrentPosition();
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
        Spinner.maxPower = maxPower;
    }
}