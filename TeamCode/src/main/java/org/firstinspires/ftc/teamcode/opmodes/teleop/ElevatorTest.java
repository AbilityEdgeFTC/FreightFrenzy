package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Elevator;

/*
 * Simple test of motion-profiled elevator autonomous operation. The elevator should move *smoothly*
 * between random heights.
 */
@Config
@Autonomous(group = "elevator")
public class ElevatorTest extends LinearOpMode {
    public static boolean moveToMin = false;
    public static boolean moveToMid = false;
    public static boolean moveToMax = false;
    public static boolean moveToZero = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Elevator elevator = new Elevator(hardwareMap);
        NanoClock clock = NanoClock.system();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            double startTime = clock.seconds();
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

            while (!isStopRequested() && (clock.seconds() - startTime) < 5 && moveToMin)
            {
                elevator.update();
                telemetry.addData("targetVelocity", elevator.getTargetVelocity());
                telemetry.addData("measuredVelocity", elevator.getVelocity());
                telemetry.update();
            }
            while (!isStopRequested() && (clock.seconds() - startTime) < 5 && moveToMid)
            {
                elevator.update();
                telemetry.addData("targetVelocity", elevator.getTargetVelocity());
                telemetry.addData("measuredVelocity", elevator.getVelocity());
                telemetry.update();
            }while (!isStopRequested() && (clock.seconds() - startTime) < 5 && moveToMax)
            {
                elevator.update();
                telemetry.addData("targetVelocity", elevator.getTargetVelocity());
                telemetry.addData("measuredVelocity", elevator.getVelocity());
                telemetry.update();
            }while (!isStopRequested() && (clock.seconds() - startTime) < 5 && moveToZero)
            {
                elevator.update();
                telemetry.addData("targetVelocity", elevator.getTargetVelocity());
                telemetry.addData("measuredVelocity", elevator.getVelocity());
                telemetry.update();
            }

            telemetry.addData("targetVelocity", elevator.getTargetVelocity());
            telemetry.addData("measuredVelocity", elevator.getVelocity());
            telemetry.update();

        }
    }
}