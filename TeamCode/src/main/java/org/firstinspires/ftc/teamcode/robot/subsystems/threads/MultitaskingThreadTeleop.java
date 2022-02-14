/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems.threads;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

@Config
public class MultitaskingThreadTeleop extends Thread {

    org.firstinspires.ftc.teamcode.robot.subsystems.intake intake;
    Gamepad gamepad1, gamepad2;
    org.firstinspires.ftc.teamcode.robot.subsystems.dip dip;
    public static double powerIntake = 1;
    ElevatorThread elevator;
    cGamepad cGamepad1, cGamepad2;
    boolean runFrontIntake = false, runBackIntake = false, canFrontIntake = false, canBackIntake = false;
    Telemetry telemetry;

    public MultitaskingThreadTeleop(HardwareMap hw, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        intake = new intake(hw);
        dip = new dip(hw);
        this.telemetry = telemetry;
        elevator = new ElevatorThread(hw, telemetry, gamepad2);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run() {
        try {
            dip.getFreight();

            while (!isInterrupted()) {
                cGamepad1.update();
                cGamepad2.update();

                if (gamepad1.left_trigger != 0 && canBackIntake)
                {
                    intake.powerIntake(-gamepad1.left_trigger);
                    runBackIntake = false;
                    runFrontIntake = false;
                }
                else if (gamepad1.right_trigger != 0 && canFrontIntake)
                {
                    intake.powerIntake(gamepad1.right_trigger);
                    runBackIntake = false;
                    runFrontIntake = true;
                }
                else if (gamepad2.left_trigger != 0 && canBackIntake)
                {
                    intake.powerIntake(-gamepad2.left_trigger);
                    runBackIntake = false;
                    runFrontIntake = false;
                }
                else if (gamepad2.right_trigger != 0 && canFrontIntake)
                {
                    intake.powerIntake(gamepad2.right_trigger);
                    runBackIntake = false;
                    runFrontIntake = false;
                }

                if (cGamepad1.rightBumperOnce() )
                {
                    runFrontIntake = !runFrontIntake;
                    runBackIntake = false;
                }
                else if (cGamepad1.leftBumperOnce() ) {
                    runBackIntake = !runBackIntake;
                    runFrontIntake = false;
                }

                if(runFrontIntake && canFrontIntake)
                {
                    intake.powerIntake(powerIntake);
                }
                else if(runBackIntake && canBackIntake)
                {
                    intake.powerIntake(-powerIntake);
                }
                else if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0 && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0)
                {
                    intake.stop();
                }

                if((elevator.elevatorSate == ElevatorThread.ElevatorState.MIN) || (elevator.elevatorSate == ElevatorThread.ElevatorState.MID) || (elevator.elevatorSate == ElevatorThread.ElevatorState.MAX) && !(cGamepad2.rightBumperOnce() || cGamepad2.leftBumperOnce()))
                {
                    dip.releaseFreightPos();
                    canFrontIntake = false;
                    canBackIntake = false;
                }
                else if(elevator.elevatorSate == ElevatorThread.ElevatorState.ZERO && !(cGamepad2.rightBumperOnce() || cGamepad2.leftBumperOnce()))
                {
                    dip.getFreight();
                    canFrontIntake = true;
                    canBackIntake = true;
                }

                if(cGamepad2.rightBumperOnce() || cGamepad2.leftBumperOnce())
                {
                    dip.releaseFreight();
                }

            }

            Thread.currentThread().interrupt();
        }
        // an error occurred in the run loop.
        catch (Exception e) {
        }
    }
}