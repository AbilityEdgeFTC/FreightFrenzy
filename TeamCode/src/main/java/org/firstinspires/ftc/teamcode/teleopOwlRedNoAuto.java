/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

@Config
@TeleOp(name = "TeleOp Red Alliance No Autonomous", group = "main")
public class teleopOwlRedNoAuto extends LinearOpMode {

    gamepad gamepad;
    ElevatorSpinnerLibraryPID spinner;
    ElevatorFirstPID elevator;
    carousel carousel;
    //IntakeFixingThread intakeFixingThread;
    intake intake;
    hand hand;
    dip dip;
    private boolean frontIntake = false, backIntake = false;
    cGamepad cGamepad1, cGamepad2;
    public static double powerIntake = 1, powerSlowElevator = .6, powerElevator = 1;
    boolean canIntake = true;
    double position = 0;
    ElapsedTime resetElevator;

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

    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;
    @Override
    public void runOpMode() throws InterruptedException {
        //        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt"), "" + Math.toRadians(startPoseRightH));
        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("ElevatorValue.txt"), "" + 0);
//        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("SpinnerValue.txt"), "" + 0);

        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        gamepad.setRedAlliance(true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        spinner = new ElevatorSpinnerLibraryPID(hardwareMap, gamepad1, gamepad2);
        elevator = new ElevatorFirstPID(hardwareMap, gamepad2);
        carousel = new carousel(hardwareMap);
        //intakeFixingThread = new IntakeFixingThread(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
        spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.ZERO_RED);
        resetElevator = new ElapsedTime();

        // wait till after init
        waitForStart();

        //intakeFixingThread.start();

        while (opModeIsActive()) {
            cGamepad1.update();
            cGamepad2.update();
            gamepad.update();
            elevator.update();
            spinner.update();
            intakeToggles();
            elevatorSwitch();
            resetElevatorMidMoving();
            pidToggles();
            carouselSpinning();
            if(withoutPID())
            {
                handMoving();
            }
            if(gamepad1.left_stick_button)
            {
                gamepad.setPower(1);
            }
            else
            {
                gamepad.setPower(gamepad.mainPower);
            }
            elevatorLeveling();
            telemetry.update();
        }

        gamepad.saveIMUHeading();


        //intakeFixingThread.exitThread();
        //intakeFixingThread.interrupt();
    }

    void elevatorLeveling()
    {
        if(gamepad2.a)
        {
            elevatorLevel = 1;
        }
        else if(gamepad2.b)
        {
            elevatorLevel = 2;
        }
        else if(gamepad2.y)
        {
            elevatorLevel = 3;
        }
        else if(gamepad2.x)
        {
            elevatorLevel = 0;
        }
    }

    void handMoving()
    {
        position = hand.getPos();

        if(cGamepad2.rightBumperOnce() && hand.getPos() <= 0.99)
        {
            position -= 0.01;
            hand.moveTo(position);
        }
        else if(cGamepad2.leftBumperOnce() && hand.getPos() > 0.01)
        {
            position += 0.01;
            hand.moveTo(position);
        }

    }

    boolean pidToggles()
    {
        if(gamepad2.left_stick_button || gamepad2.right_stick_y != 0 || gamepad2.right_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0 && !gamepad2.right_stick_button)
        {
            spinner.setUsePID(false);
            elevator.setUsePID(false);
            return false;
        }
        else if(gamepad2.right_stick_button || gamepad1.left_bumper && !gamepad2.left_stick_button && elevatorMovement != ElevatorMovement.SPIN)
        {
            spinner.setUsePID(true);
            elevator.setUsePID(true);
        }

        return true;
    }

    void carouselSpinning()
    {
        if(gamepad2.dpad_right)
        {
            carousel.spin(false, true);
        }
        else if(gamepad2.dpad_left)
        {
            carousel.spin(true, true);
        }
        else
        {
            carousel.stop();
        }
    }

    void intakeToggles()
    {
        if ((gamepad1.right_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad2.left_trigger == 0) && (gamepad1.left_trigger == 0)/* && !intakeFixingThread.withFreight()*/)
        {
            intake.powerIntake(gamepad1.right_trigger);
            frontIntake = false;
            backIntake = false;
            resetSpinner();
        }
        else if ((gamepad1.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad2.left_trigger == 0)/* && !intakeFixingThread.withFreight()*/)
        {
            intake.powerIntake(-gamepad1.left_trigger);
            frontIntake = false;
            backIntake = false;
            resetSpinner();
        }
        else if ((gamepad2.right_trigger != 0) && canIntake && (gamepad2.left_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            //intakeFixingThread.setOverride(true);
            intake.powerIntake(gamepad2.right_trigger);
            frontIntake = false;
            backIntake = false;
            resetSpinner();
        }
        else if ((gamepad2.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            //intakeFixingThread.setOverride(true);
            intake.powerIntake(-gamepad2.left_trigger);
            frontIntake = false;
            backIntake = false;
            resetSpinner();
        }

        //intakeFixingThread.setOverride(false);

        powerIntake();
    }

    void powerIntake()
    {
        if(frontIntake && canIntake /*&& !intakeFixingThread.withFreight()*/)
        {
            intake.powerIntake(powerIntake);

        }
        else if(backIntake && canIntake)
        {
            intake.powerIntake(-powerIntake);
        }
        else if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0 && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0)
        {
            intake.stop();
        }

    }

    void resetSpinner()
    {
        spinner.setUsePID(true);
    }

    void elevatorSwitch() {
        switch (elevatorMovement) {
            case SPIN:
                resetElevator();
                elevator.update();
                spinner.update();
                spinner.setSlowMove(false);
                if(withoutPID())
                {
                    handMoving();
                }
                gamepad.setCanTwist(true);

                if (gamepad1.right_bumper)
                {
                    spinner.setUsePID(true);
                    spinner.setSlowMove(true);
                    gamepad.setCanTwist(false);
                    powerElevator = powerSlowElevator;
                    elevator.setPower(powerElevator);
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;
                    //intakeFixingThread.setOverride(true);
                    if(!withoutPID())
                    {
                        dip.holdFreight();
                    }

                    switch (elevatorLevel)
                    {
                        case 0:
                            spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.SHARED_RED);
                            elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                            elevatorMovement = ElevatorMovement.SHARED;
                            break;
                        case 1:
                            spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                            elevatorMovement = ElevatorMovement.LEVEL1;
                            break;
                        case 2:
                            spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                            elevatorMovement = ElevatorMovement.LEVEL2;
                            break;
                        case 3:
                            spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                            elevatorMovement = ElevatorMovement.LEVEL3;
                            break;
                        default:
                            elevator.setUsePID(false);
                    }
                }
                break;
            case LEVEL1:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                if(!withoutPID())
                {
                    hand.level1();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL2:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                if(!withoutPID())
                {
                    hand.level2();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL3:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                if(!withoutPID())
                {
                    hand.level3();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case SHARED:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                if(!withoutPID())
                {
                    hand.shared();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case DIP:
                elevator.update();
                spinner.update();
                spinner.setUsePID(false);
                if(withoutPID())
                {
                    handMoving();
                }
                if(gamepad1.right_stick_button)
                {
                    gamepad.setCanTwist(false);
                    spinner.setSlowMove(true);
                }
                else
                {
                    gamepad.setCanTwist(true);
                    spinner.setSlowMove(false);
                }

                if(gamepad2.right_trigger == 1 && gamepad2.left_trigger == 1 && elevator.getUsePID() == false && spinner.getUsePID() == false)
                {
                    dip.releaseFreight();
                }

                if(gamepad1.left_bumper)
                {
                    if(!withoutPID())
                    {
                        dip.releaseFreight();
                    }

                    gamepad.setCanTwist(true);
                    resetElevator.reset();

                    //intakeFixingThread.setOverride(false);
                    canIntake = true;
                    frontIntake = true;

                    if(spinner.getSpinnerState() == ElevatorSpinnerLibraryPID.SpinnerState.SHARED_RED)
                    {
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                        resetElevator.reset();
                    }

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

        if(!withoutPID())
        {
            dip.releaseFreight();
        }

        elevator.setPower(powerSlowElevator);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);

        if(!withoutPID())
        {
            hand.intake();
        }

        switch (elevatorLevel)
        {
            case 0:
                hand.intake();
                if(resetElevator.seconds() > .67 && pidToggles())
                {
                    elevator.setUsePID(true);
                    spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.SHARED_RED);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 1:
                hand.intake();
                if(resetElevator.seconds() > 1.3 && pidToggles())
                {
                    elevator.setUsePID(true);
                    spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 2:
                hand.intake();
                if(resetElevator.seconds() > .8 && pidToggles())
                {
                    elevator.setUsePID(true);
                    spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 3:
                if(resetElevator.seconds() > .6 && pidToggles())
                {
                    if(!withoutPID())
                    {
                        hand.intake();
                    }
                    elevator.setUsePID(true);
                    spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
        }

        canIntake = true;
    }

    void resetElevatorMidMoving()
    {
        if(gamepad1.left_bumper && elevatorMovement != ElevatorMovement.SPIN)
        {
            elevatorMovement = ElevatorMovement.SPIN;
            //intakeFixingThread.setOverride(false);
            canIntake = true;
            frontIntake = true;
        }

    }

    boolean withoutPID()
    {
        if(elevator.getUsePID() == true || spinner.getUsePID() == true && elevator.getElevatorLevel() != ElevatorFirstPID.ElevatorLevel.ZERO)
        {
            switch (hand.getHandPos())
            {
                case SHARED_HUB:
                    hand.shared();
                    break;
                case INTAKE:
                    //hand.intake();
                    break;
                case ONE_HUB:
                    hand.level1();
                    break;
                case TWO_HUB:
                    hand.level2();
                    break;
                case THREE_HUB:
                    hand.level3();
                    break;
            }
            return false;
        }
        else
        {
            return true;
        }
    }


}