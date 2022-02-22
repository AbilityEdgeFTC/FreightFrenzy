/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.threads.IntakeFixingThread;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.MID_HEIGHT;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.ZERO_HEIGHT;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.ZERO_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.usePID;

@Config
@TeleOp(group = "main")
public class teleopOwl extends LinearOpMode {

    gamepad gamepad;
    ElevatorSpinner spinner;
    Elevator elevator;
    carousel carousel;
    //IntakeFixingThread intake;
    //Thread intakeFixingThread;
    intake intake;
    //hand hand;
    //dip dip;
    boolean retract = false;
    private boolean frontIntake = false, backIntake = false;
    cGamepad cGamepad1;
    public static double powerIntake = 1;
    boolean canIntake = true;

    enum ElevatorMovement
    {
        SPIN,
        EXTEND_LEVEL1,
        EXTEND_LEVEL2,
        EXTEND_LEVEL3,
        DIP
    }

    public static int ElevatorLevel = 0;
    public static int SpinnerLevel = 1;

    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        spinner = new ElevatorSpinner(hardwareMap, gamepad2);
        elevator = new Elevator(hardwareMap, gamepad2);
        carousel = new carousel(hardwareMap);
        //intake = new IntakeFixingThread(hardwareMap, telemetry);
        //intakeFixingThread = intake;
        intake = new intake(hardwareMap);
        //hand = new hand(hardwareMap);
        //dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);

        // wait till after init
        waitForStart();

        //intakeFixingThread.start();

        while (opModeIsActive()) {
            cGamepad1.update();

            /*if(gamepad1.a)
            {
                ElevatorLevel = 0;
            }
            else if(gamepad1.x)
            {
                ElevatorLevel = 2;
            }
            else if(gamepad1.b)
            {
                ElevatorLevel = 1;
            }

            if(gamepad2.a)
            {
                SpinnerLevel = 0;
            }
            else if(gamepad2.x)
            {
                SpinnerLevel = 2;
            }
            else if(gamepad2.b)
            {
                SpinnerLevel = 1;
            }*/

            if(gamepad2.left_stick_button)
            {
                spinner.usePID = false;
                elevator.usePID = false;
            }
            else if(gamepad2.right_stick_button)
            {
                spinner.usePID = true;
                elevator.usePID = true;
            }

            if ((gamepad1.left_trigger != 0 || gamepad2.left_trigger != 0) && canIntake)
            {
                intake.powerIntake(-gamepad1.left_trigger);
                frontIntake = false;
                backIntake = false;
            }
            else if ((gamepad1.right_trigger != 0 || gamepad2.right_trigger != 0) && canIntake)
            {
                intake.powerIntake(gamepad1.right_trigger);
                frontIntake = false;
                backIntake = false;
            }

            if(frontIntake && canIntake)
            {
                intake.powerIntake(powerIntake);
            }
            else if(backIntake && canIntake)
            {
                intake.powerIntake(-powerIntake);
            }
            else if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0)
            {
                intake.stop();
            }


            switch (elevatorMovement) {
                case SPIN:
                    spinner.target = ZERO_ANGLE;
                    elevator.target = ZERO_HEIGHT;
                    //dip.getFreight();
                    retract = false;
                    canIntake = true;

                    if (gamepad1.right_bumper)
                    {
                        //intake.spinIntake = false;
                        frontIntake = false;
                        backIntake = false;
                        canIntake = false;
                        //dip.holdFreight();

                        switch (SpinnerLevel)
                        {
                            case 0:
                                spinner.target = MIN_ANGLE;
                                break;
                            case 2:
                                spinner.target = MAX_ANGLE;
                                break;
                            default:
                                spinner.usePID = false;
                                elevator.usePID = false;
                        }

                        switch (ElevatorLevel)
                        {
                            case 0:
                                elevatorMovement = ElevatorMovement.EXTEND_LEVEL1;
                                break;
                            case 1:
                                elevatorMovement = ElevatorMovement.EXTEND_LEVEL2;
                                break;
                            case 2:
                                elevatorMovement = ElevatorMovement.EXTEND_LEVEL3;
                                break;
                        }

                    }
                    break;
                case EXTEND_LEVEL1:
                    elevator.target = MIN_HEIGHT;
                    //hand.level1();
                    elevatorMovement = ElevatorMovement.DIP;
                    break;
                case EXTEND_LEVEL2:
                    elevator.target = MID_HEIGHT;
                    //hand.level2();
                    elevatorMovement = ElevatorMovement.DIP;
                    break;
                case EXTEND_LEVEL3:
                    elevator.target = MAX_HEIGHT;
                    //hand.level3();
                    elevatorMovement = ElevatorMovement.DIP;
                    break;
                case DIP:
                    if(gamepad1.left_bumper || retract)
                    {
                        //dip.releaseFreight();
                        //intake.spinIntake = true;
                        canIntake = true;
                        frontIntake = true;
                        elevatorMovement = ElevatorMovement.SPIN;
                    }
                    break;
                default:
                    elevatorMovement = ElevatorMovement.SPIN;
                    break;
            }

            if(gamepad1.left_bumper && elevatorMovement != ElevatorMovement.SPIN)
            {
                elevatorMovement = ElevatorMovement.SPIN;
                //intake.spinIntake = true;
                canIntake = true;
                frontIntake = true;
            }

            gamepad.update();
            elevator.update();
            spinner.update();
            telemetry.update();
        }

        //intake.exitThread();
        //intakeFixingThread.interrupt();
    }

}