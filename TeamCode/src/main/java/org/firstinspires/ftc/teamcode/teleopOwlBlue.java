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
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorLibraryPID.SHARED_HUB;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorLibraryPID.ZERO_HEIGHT;

@Config
@TeleOp(name = "TeleOp Blue Alliance", group = "main")
public class teleopOwlBlue extends LinearOpMode {

    gamepad gamepad;
    ElevatorSpinnerLibraryPID spinner;
    ElevatorFirstPID elevator;
    carousel carousel;
    //IntakeFixingThread intake;
    //Thread intakeFixingThread;
    intake intake;
    hand hand;
    dip dip;
    private boolean frontIntake = false, backIntake = false;
    cGamepad cGamepad1, cGamepad2;
    public static double powerIntake = 1, powerSlowElevator = .6, powerElevator = 1, powerSlowElevatorHub = .1;
    boolean canIntake = true;
    double position = 0;
    ElapsedTime sharedTime;

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
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        spinner = new ElevatorSpinnerLibraryPID(hardwareMap, gamepad2);
        elevator = new ElevatorFirstPID(hardwareMap, gamepad2);
        carousel = new carousel(hardwareMap);
        //intake = new IntakeFixingThread(hardwareMap, telemetry);
        //intakeFixingThread = intake;
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
        spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.ZERO_BLUE);
        sharedTime = new ElapsedTime();

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
            if(withPID())
            {
                handMoving();
            }
            elevatorLeveling();
            //telemetry.addData("Sec", sharedTime.seconds());
            telemetry.update();
        }

        gamepad.saveIMUHeading();

        //intake.exitThread();
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

        if(cGamepad2.rightBumperOnce() && hand.getPos() <= 0.95)
        {
            position += 0.05;
            hand.moveTo(position);
        }
        else if(cGamepad2.leftBumperOnce() && hand.getPos() > 0.05)
        {
            position -= 0.05;
            hand.moveTo(position);
        }
    }

    void pidToggles()
    {
        if(gamepad2.left_stick_button)
        {
            spinner.setUsePID(false);
            elevator.setUsePID(false);
        }
        else if(gamepad2.right_stick_button || gamepad1.left_bumper)
        {
            spinner.setUsePID(true);
            elevator.setUsePID(true);
            switch (hand.getHandPos())
            {
                case SHARED_HUB:
                    hand.shared();
                    break;
                case INTAKE:
                    hand.intake();
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
        }
    }

    void carouselSpinning()
    {
        if(gamepad2.dpad_right)
        {
            carousel.spin(false);
        }
        else if(gamepad2.dpad_left)
        {
            carousel.spin(true);
        }
        else
        {
            carousel.stop();
        }
    }

    void intakeToggles()
    {
        if ((gamepad1.right_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad2.left_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(gamepad1.right_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad1.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad2.left_trigger == 0))
        {
            intake.powerIntake(-gamepad1.left_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad2.right_trigger != 0) && canIntake && (gamepad2.left_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(gamepad2.right_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad2.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(-gamepad2.left_trigger);
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
        else if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0 && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0)
        {
            intake.stop();
        }
    }

    void elevatorSwitch() {
        switch (elevatorMovement) {
            case SPIN:
                resetElevator();
                elevator.update();
                spinner.update();
                if(withPID())
                {
                    handMoving();
                }

                if (gamepad1.right_bumper)
                {
                    powerElevator = powerSlowElevator;
                    elevator.setPower(powerElevator);
                    //intake.spinIntake = false;
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;
                    dip.holdFreight();

                    switch (elevatorLevel)
                    {
                        case 0:
                            spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.SHARED_BLUE);
                            elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
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
                        default:
                            elevator.setUsePID(false);
                    }
                }
                break;
            case LEVEL1:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.LEFT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                if(!withPID())
                {
                    hand.level1();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL2:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.LEFT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                if(!withPID())
                {
                    hand.level2();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL3:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.LEFT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                if(!withPID())
                {
                    hand.level3();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case SHARED:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.SHARED_BLUE);
                if(!withPID())
                {
                    hand.shared();
                }
                sharedTime.reset();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case DIP:
                elevator.update();
                spinner.update();
                if(withPID())
                {
                    handMoving();
                }

                if(gamepad1.left_bumper)
                {
                    dip.releaseFreight();

                    if(!withPID())
                    {
                        hand.intake();
                    }

                    //intake.spinIntake = true;
                    canIntake = true;
                    frontIntake = true;

                    if(spinner.getSpinnerState() == ElevatorSpinnerLibraryPID.SpinnerState.SHARED_BLUE)
                    {
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                        sharedTime.reset();
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

        dip.getFreight();
        if(!withPID())
        {
            hand.intake();
        }

        elevator.setPower(powerSlowElevator);
        switch (elevatorLevel)
        {
            case 0:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.SHARED_BLUE);
                if(sharedTime.seconds() > .67)
                {
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 3:
            case 2:
            case 1:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.LEFT);
                break;
        }

        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);

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

    boolean withPID()
    {
        if(elevator.getUsePID() == true || spinner.getUsePID() == true )
        {
            switch (hand.getHandPos())
            {
                case SHARED_HUB:
                    hand.shared();
                    break;
                case INTAKE:
                    hand.intake();
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