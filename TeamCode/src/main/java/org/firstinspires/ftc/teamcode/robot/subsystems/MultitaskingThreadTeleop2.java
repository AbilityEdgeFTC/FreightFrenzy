/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.transportation;
import org.firstinspires.ftc.teamcode.robot.subsystems.tse;

@Config
public class MultitaskingThreadTeleop2 extends Thread {

    org.firstinspires.ftc.teamcode.robot.subsystems.tse tse;
    Gamepad gamepad1,gamepad2;
    transportation transportation;
    int counter = 1;
    cGamepad gamepad;

    public MultitaskingThreadTeleop2(HardwareMap hw, Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {
        tse = new tse(hw);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        transportation = new transportation(hw);
        gamepad = new cGamepad(gamepad1);
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run() {
        try {
            while (!isInterrupted()) {
                gamepad.update();

                if(gamepad2.y){
                    tse.goToPos3();
                }
                else if(gamepad2.b){
                    tse.goToPos2();
                }
                else if(gamepad2.a){
                    tse.goToPos1();
                }


                if(gamepad.dpadUpOnce() && counter < 4)
                {
                    counter++;
                }
                else if(gamepad.dpadDownOnce() && counter > 1)
                {
                    counter--;
                }

                switch (counter)
                {
                    case 1:
                        transportation.moveToPos1();
                        break;
                    case 2:
                        transportation.moveToPos2();
                        break;
                    case 3:
                        transportation.moveToPos3();
                        break;
                    case 4:
                        transportation.moveToPos4();
                        break;
                }
            }
        }
        // an error occurred in the run loop.
        catch (Exception e) {
        }
    }

}