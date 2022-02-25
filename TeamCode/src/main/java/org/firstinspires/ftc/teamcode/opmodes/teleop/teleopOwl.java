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
import org.firstinspires.ftc.teamcode.robot.subsystems.hand.HandPos;
import org.firstinspires.ftc.teamcode.robot.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.threads.IntakeFixingThread;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL1;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL2;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL3;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.SHARED_HUB;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.ZERO_HEIGHT;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.elevatorLevel;
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
    hand hand;
    dip dip;
    private boolean frontIntake = false, backIntake = false;
    cGamepad cGamepad1;
    public static double powerIntake = 1;
    boolean canIntake = true;

    enum ElevatorMovement
    {
        SPIN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        SHARED,
        DIP
    }

    public static int elevatorLevel = 0;
    public static int spinnerLevel = 0;

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
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);

        // wait till after init
        waitForStart();

        //intakeFixingThread.start();

        while (opModeIsActive()) {
            cGamepad1.update();
            hand.goToPositions();

            /*if(gamepad1.a)
            {
                elevatorLevel = 0;
            }
            else if(gamepad1.x)
            {
                elevatorLevel = 2;
            }
            else if(gamepad1.b)
            {
                elevatorLevel = 1;
            }

            if(gamepad2.a)
            {
                SpinnerLevel = 0;
            }
            else if(gamepad2.x)
            {
                SpinnerLevel = 2;
            }*/

            if(gamepad2.left_stick_button)
            {
                spinner.setUsePID(false);
                elevator.setUsePID(false);
            }
            else if(gamepad2.right_stick_button)
            {
                spinner.setUsePID(true);
                elevator.setUsePID(true);
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
                    spinner.setTarget(ZERO_ANGLE);
                    elevator.setTarget(ZERO_HEIGHT);
                    dip.getFreight();
                    hand.setHandPos(HandPos.INTAKE);
                    canIntake = true;

                    if (gamepad1.right_bumper)
                    {
                        //intake.spinIntake = false;
                        frontIntake = false;
                        backIntake = false;
                        canIntake = false;
                        dip.holdFreight();

                        switch (spinnerLevel)
                        {
                            case 0:
                                spinner.setTarget(MIN_ANGLE);
                                break;
                            case 2:
                                spinner.setTarget(MAX_ANGLE);
                                break;
                            default:
                                spinner.setUsePID(false);
                                elevator.setUsePID(false);
                        }

                        switch (elevatorLevel)
                        {
                            case 0:
                                elevatorMovement = ElevatorMovement.SHARED;
                                break;
                            case 1:
                                elevatorMovement = ElevatorMovement.LEVEL1;
                                break;
                            case 2:
                                elevatorMovement = ElevatorMovement.LEVEL2;
                                break;
                            case 3:
                                elevatorMovement = ElevatorMovement.LEVEL3;
                                break;
                        }

                    }
                    break;
                case LEVEL1:
                    elevator.setTarget(HUB_LEVEL1);
                    elevatorMovement = ElevatorMovement.DIP;
                    hand.setHandPos(HandPos.ONE_HUB);
                    break;
                case LEVEL2:
                    elevator.setTarget(HUB_LEVEL2);
                    elevatorMovement = ElevatorMovement.DIP;
                    hand.setHandPos(HandPos.TWO_HUB);
                    break;
                case LEVEL3:
                    elevator.setTarget(HUB_LEVEL3);
                    elevatorMovement = ElevatorMovement.DIP;
                    hand.setHandPos(HandPos.THREE_HUB);
                    break;
                case SHARED:
                    elevator.setTarget(SHARED_HUB);
                    elevatorMovement = ElevatorMovement.DIP;
                    hand.setHandPos(HandPos.SHARED_HUB);
                    break;
                case DIP:
                    if(elevator.getTarget() == SHARED_HUB)
                    {
                        elevator.setTarget(ZERO_HEIGHT);
                    }
                    
                    if(gamepad1.left_bumper)
                    {
                        dip.releaseFreight();
                        hand.setHandPos(HandPos.INTAKE);
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

        gamepad.saveIMUHeading();

        //intake.exitThread();
        //intakeFixingThread.interrupt();
    }

}