/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto.ElevatorState;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ServoThreadAuto extends Thread {

    intake intake;
    dip dip;
    public static double powerIntake = 1;
    ElevatorThreadAuto elevator;
    boolean frontIntake = false, backIntake = false;

    public ElevatorState elevatorSate = elevator.elevatorState;

    public enum IntakeState
    {
        REVERSE,
        FORWARD,
        STOP
    }

    public static IntakeState intakeState = IntakeState.STOP;

    public ServoThreadAuto(HardwareMap hw) throws InterruptedException {
        intake = new intake(hw);
        dip = new dip(hw);
        elevator = new ElevatorThreadAuto(hw);
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run() {
        try {
            dip.getFreight();

            while (!isInterrupted()) {
                elevatorSate = elevator.elevatorState;

                if (intakeState == IntakeState.FORWARD)
                {
                    frontIntake = true;
                    backIntake = false;
                }
                else if (intakeState == IntakeState.REVERSE)
                {
                    backIntake = true;
                    frontIntake = false;
                }
                else if (intakeState == IntakeState.STOP)
                    backIntake = false;
                    frontIntake = false;
                }


                if(frontIntake)
                {
                    intake.powerIntake(powerIntake);
                }
                else if(backIntake)
                {
                    intake.powerIntake(-powerIntake);
                }
                else if(!frontIntake && !backIntake)
                {
                    intake.stop();
                }

                if((elevatorSate == elevator.elevatorState.MIN) || (elevatorSate == elevator.elevatorState.MID) || (elevatorSate == elevator.elevatorState.MAX))
                {
                    dip.releaseFreightPos();
                }
                else if((elevatorSate == elevator.elevatorState.ZERO))
                {
                    dip.getFreight();
                }

                if((elevatorSate == elevator.elevatorState.RELEASE))
                {
                    dip.releaseFreight();
                }
            }
            catch (Exception e) {
            }
    }
}