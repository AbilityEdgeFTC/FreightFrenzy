package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@TeleOp(group = "drive")
public class Spinner extends LinearOpMode {

    BasicPID PID;
    AngleController controller;
    public static double kP = 5;
    public static double kI = 0;
    public static double kD = 0;
    public static double target = 0;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 384.5 * GEAR_RATIO;
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        PID = new BasicPID(new PIDCoefficients(kP, kI, kD));
        controller = new AngleController(PID);

        motor = hardwareMap.get(DcMotorEx.class, "mS");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive())
        {
            motor.setPower(controller.calculate(Math.toRadians(target), encoderTicksToRadians(motor.getCurrentPosition())));

            telemetry.addData("Current", motor.getCurrentPosition());
            telemetry.update();
        }


    }

    public static double encoderTicksToRadians(int ticks) {
        return Math.toRadians((ticks * 360) / TICKS_PER_REV);
    }

}
