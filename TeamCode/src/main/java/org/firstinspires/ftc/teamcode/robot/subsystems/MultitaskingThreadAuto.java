/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakeState;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.robot.subsystems.intake.intakeState;

@Config
public class MultitaskingThreadAuto extends Thread {

    intake intake;
    dip dip;
    public static double powerIntake = 1;
    ElevatorThreadAuto elevator;
    public static boolean frontIntake = false, backIntake = false;

    public MultitaskingThreadAuto(HardwareMap hw) {
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

                if((ElevatorThreadAuto.elevatorState == ElevatorThreadAuto.ElevatorState.MIN) || (ElevatorThreadAuto.elevatorState == ElevatorThreadAuto.ElevatorState.MID) || (ElevatorThreadAuto.elevatorState == ElevatorThreadAuto.ElevatorState.MAX))
                {
                    dip.releaseFreightPos();
                    dip.handState = dip.handState.HOLD;
                }
                else if(ElevatorThreadAuto.elevatorState == ElevatorThreadAuto.ElevatorState.ZERO)
                {
                    dip.getFreight();
                    dip.handState = dip.handState.HOLD;
                }

                if(dip.handState == dip.handState.RELEASE)
                {
                    dip.releaseFreight();
                }
            } catch (InterruptedException interruptedException) {
            interruptedException.printStackTrace();
        }
    }
}