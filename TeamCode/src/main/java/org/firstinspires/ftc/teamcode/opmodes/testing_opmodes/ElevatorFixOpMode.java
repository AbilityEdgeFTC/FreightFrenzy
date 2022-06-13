package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@TeleOp(name = "Elevator Fixing", group = "testing")
public class ElevatorFixOpMode extends LinearOpMode {

    public static double HUB_LEVEL3 = 23;
    public static double TICKS_PER_REV = 145.1;
    public static double SPOOL_RADIUS = 0.75; // in
    public static int offset = 0;
    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {

        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive())
        {
            motor.setTargetPosition(inchesToEncoderTicks(HUB_LEVEL3));
            motor.setPower(0.35);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);
            motor.setTargetPosition(inchesToEncoderTicks(0));
            motor.setPower(0.35);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


    }
    int inchesToEncoderTicks(double inches) {
        return (int)Math.round((inches * TICKS_PER_REV) / (SPOOL_RADIUS * 2 * Math.PI));
    }
}
