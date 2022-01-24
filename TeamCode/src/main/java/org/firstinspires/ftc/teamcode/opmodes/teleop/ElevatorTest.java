package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Subsystems.Elevator;

/*
 * Simple test of motion-profiled elevator autonomous operation. The elevator should move *smoothly*
 * between random heights.
 */
@Config
@TeleOp(group = "elevator")
@Disabled
public class ElevatorTest extends LinearOpMode {
    public static boolean moveToMin = false;
    public static boolean moveToMid = false;
    public static boolean moveToMax = false;
    public static boolean moveToZero = false;

    public static double MAX_HEIGHT = 15.5; // TODO set value in inches
    public static double MID_HEIGHT = 9; // TODO set value in inches
    public static double MIN_HEIGHT = 4; // TODO set value in inches
    public static double ZERO_HEIGHT = 0; // TODO set value in inches

    public static double timeTo = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        Elevator elevator = new Elevator(hardwareMap, MAX_HEIGHT, MID_HEIGHT, MIN_HEIGHT, ZERO_HEIGHT);
        NanoClock clock = NanoClock.system();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double startTime = clock.seconds();
            checkLevel();

            if(moveToMin)
            {
                elevator.setHeight(Elevator.MIN_HEIGHT);
            }
            else if(moveToMid)
            {
                elevator.setHeight(Elevator.MID_HEIGHT);
            }
            else if(moveToMax)
            {
                elevator.setHeight(Elevator.MAX_HEIGHT);
            }
            else if(moveToZero)
            {
                elevator.setHeight(Elevator.ZERO_HEIGHT);
            }

            while (!isStopRequested() && (clock.seconds() - startTime) < timeTo && moveToMin)
            {
                elevator.update();
                telemetry.addData("targetVelocity", elevator.getTargetVelocity());
                telemetry.addData("measuredVelocity", elevator.getVelocity());
                telemetry.update();
                checkLevel();
                //moveToMin = false;
            }
            while (!isStopRequested() && (clock.seconds() - startTime) < timeTo && moveToMid)
            {
                elevator.update();
                telemetry.addData("targetVelocity", elevator.getTargetVelocity());
                telemetry.addData("measuredVelocity", elevator.getVelocity());
                telemetry.update();
                checkLevel();
                //moveToMid = false;
            }while (!isStopRequested() && (clock.seconds() - startTime) < timeTo && moveToMax)
            {
                elevator.update();
                telemetry.addData("targetVelocity", elevator.getTargetVelocity());
                telemetry.addData("measuredVelocity", elevator.getVelocity());
                telemetry.update();
                checkLevel();
                //moveToMax = false;
            }while (!isStopRequested() && (clock.seconds() - startTime) < timeTo && moveToZero)
            {
                elevator.update();
                telemetry.addData("targetVelocity", elevator.getTargetVelocity());
                telemetry.addData("measuredVelocity", elevator.getVelocity());
                telemetry.update();
                checkLevel();
                //moveToZero = false;
            }

            telemetry.addData("targetVelocity", elevator.getTargetVelocity());
            telemetry.addData("measuredVelocity", elevator.getVelocity());
            telemetry.update();
        }
    }

    void checkLevel()
    {
        if(gamepad1.a)
        {
            moveToMin = true;
            moveToMid = false;
            moveToMax = false;
            moveToZero = false;
        }
        else if(gamepad1.b)
        {
            moveToMid = true;
            moveToMin = false;
            moveToMax = false;
            moveToZero = false;
        }
        else if(gamepad1.y)
        {
            moveToMax = true;
            moveToMin = false;
            moveToMid = false;
            moveToZero = false;
        }
        else if(gamepad1.x)
        {
            moveToZero = true;
            moveToMin = false;
            moveToMid = false;
            moveToMax = false;
        }
    }
}