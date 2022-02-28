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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerCOMPLEX_UNSTABLE;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand.HandPos;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;

import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL1;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL2;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL3;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.SHARED_HUB;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.ZERO_HEIGHT;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.elevatorLevel;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.ZERO_ANGLE;

@Config
@TeleOp(group = "main")
public class teleopOwl extends LinearOpMode {

    gamepad gamepad;
    //ElevatorSpinnerCOMPLEX_UNSTABLE spinner;
    ElevatorSpinner spinner;
    Elevator elevator;
    carousel carousel;
    //IntakeFixingThread intake;
    //Thread intakeFixingThread;
    intake intake;
    hand hand;
    dip dip;
    DcMotor mFL, mBL, mFR, mBR;

    double leftPower_f;
    double leftPower_b;
    double rightPower_f;
    double rightPower_b;

    public double power = .7;
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

    public static int elevatorLevel = 3;
    public static int spinnerLevel = 2;

    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        //spinner = new ElevatorSpinnerCOMPLEX_UNSTABLE(hardwareMap, gamepad2);
        spinner = new ElevatorSpinner(hardwareMap, gamepad2);
        elevator = new Elevator(hardwareMap, gamepad2);
        carousel = new carousel(hardwareMap);
        //intake = new IntakeFixingThread(hardwareMap, telemetry);
        //intakeFixingThread = intake;
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);



        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mFR = hardwareMap.get(DcMotor.class, "mFR");

        mFL.setDirection(DcMotor.Direction.REVERSE);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        // wait till after init
        waitForStart();

        //intakeFixingThread.start();

        while (opModeIsActive()) {
            cGamepad1.update();
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x * 0.7;

            leftPower_f = Range.clip(drive + twist + strafe, -power, power);
            leftPower_b = Range.clip(drive + twist - strafe, -power, power);
            rightPower_f = Range.clip(drive - twist - strafe, -power, power);
            rightPower_b = Range.clip(drive - twist + strafe, -power, power);


            gamepad.update();
            elevator.update();
            spinner.update();
            telemetry.update();

            /**Gamepad 1:**/

            intakeToggles();
            elevatorSwitch();
            resetElevatorMidMoving();

            mFL.setPower(leftPower_f);
            mBL.setPower(leftPower_b);
            mFR.setPower(rightPower_f);
            mBR.setPower(rightPower_b);

            /**Gamepad 2:**/
        }


        gamepad.saveIMUHeading();

        //intake.exitThread();
        //intakeFixingThread.interrupt();
    }

    void pidToggles()
    {
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
    }

    void intakeToggles()
    {
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


        powerIntake();
    }

    void powerIntake()
    {
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
    }

    void elevatorSwitch()
    {
        switch (elevatorMovement) {
            case SPIN:
                resetElevator();
                elevator.update();
                spinner.update();

                if (gamepad1.right_bumper)
                {
                    //intake.spinIntake = false;
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;
                    hand.intake();
                    dip.holdFreight();

                    spinner.setSpinnerState(ElevatorSpinner.SpinnerState.RIGHT);
                    elevatorMovement = ElevatorMovement.LEVEL3;
//                    switch (spinnerLevel)
//                    {
//                        case 0:
//                            spinner.setTarget(MIN_ANGLE);
//                            break;
//                        case 2:
//                            spinner.setTarget(MAX_ANGLE);
//                            break;
//                        default:
//                            spinner.setUsePID(false);
//                    }
//
//                    switch (elevatorLevel)
//                    {
//                        case 0:
//                            elevatorMovement = ElevatorMovement.SHARED;
//                            break;
//                        case 1:
//                            elevatorMovement = ElevatorMovement.LEVEL1;
//                            break;
//                        case 2:
//                            elevatorMovement = ElevatorMovement.LEVEL2;
//                            break;
//                        case 3:
//                            elevatorMovement = ElevatorMovement.LEVEL3;
//                            break;
//                        default:
//                            elevator.setUsePID(false);
//                    }


                }
                break;
            case LEVEL1:
                elevator.setElevatorLevel(Elevator.ElevatorLevel.HUB_LEVEL1);
                hand.level1();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL2:
                elevator.setElevatorLevel(Elevator.ElevatorLevel.HUB_LEVEL2);
                hand.level2();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL3:
                elevator.setElevatorLevel(Elevator.ElevatorLevel.HUB_LEVEL3);
                hand.level3();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case SHARED:
                elevator.setElevatorLevel(Elevator.ElevatorLevel.SHARED_HUB);
                hand.shared();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case DIP:
                if(elevator.getTarget() == SHARED_HUB)
                {
                    elevator.setElevatorLevel(Elevator.ElevatorLevel.ZERO);
                }

                elevator.update();
                spinner.update();

                if(gamepad1.left_bumper)
                {
                    dip.releaseFreight();
                    if(elevator.getTarget() == ZERO_HEIGHT)
                    {
                        elevator.setElevatorLevel(Elevator.ElevatorLevel.SHARED_HUB);
                    }
                    hand.intake();
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
    }

    void resetElevator()
    {
        dip.getFreight();
        hand.intake();
        elevator.setElevatorLevel(Elevator.ElevatorLevel.ZERO);
        spinner.setSpinnerState(ElevatorSpinner.SpinnerState.ZERO);
        canIntake = true;
    }

    void resetElevatorMidMoving()
    {
        if(gamepad1.left_bumper && elevatorMovement != ElevatorMovement.SPIN)
        {
            elevatorMovement = ElevatorMovement.SPIN;
            //intake.spinIntake = true;
            canIntake = true;
            frontIntake = true;
        }

    }

}