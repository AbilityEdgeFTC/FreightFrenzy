package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robot.subsystems.Cover;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.FreightSensor;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

import java.util.Arrays;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Auto - Right Red NEW", group = "Autonomous Red")
public class AutoRightRedAsyncNew extends LinearOpMode {

    double startPoseRightX = 12.7;
    double startPoseRightY = -72 + 17.72;
    double startPoseRightH = 90;

    public static double poseEntranceX = 8;
    public static double poseEntranceY = -59;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 49.5;
    public static double poseCollectY = -58;
    public static double poseCollectH = 180;
    public static double poseHelpX = 7;
    public static double poseHelpY = -50;
    public static double poseHelpH = 180;
    public static double poseCollectX2 = 60;
    public static double poseCollectY2 = -58;
    public static double poseCollectH2 = 180;

    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    intake intake;
    dip dip;
    Cover cover;
    ElapsedTime runningFor;
    FreightSensor freightSensor;
    SampleMecanumDrive drive;

    public static double powerSlowElevator = .7, powerElevator = 1, powerElevatorFast = 1, elevatorDelay = .9;

    boolean hasFreight = false, firstTime = true;
    double offset = 0;

    TrajectorySequence fixAngle, goToIntake1, goToHub2, goToIntake2, goToHub3, goToIntake3, goToHub4, goToPark, goToHub5, goToIntake4, goToIntake5;
    Pose2d startPoseRight, poseHelp, poseEntrance, poseCollect, poseCollectCycle2;

    enum State
    {
        FIX_ANGLE,
        OPEN_ELEVATOR1,
        WAIT_ELEVATOR_DELAY1,
        INTAKE1,
        OPEN_ELEVATOR2,
        WAIT_ELEVATOR_DELAY2,
        INTAKE2,
        OPEN_ELEVATOR3,
        WAIT_ELEVATOR_DELAY3,
        INTAKE3,
        OPEN_ELEVATOR4,
        WAIT_ELEVATOR_DELAY4,
        INTAKE4,
        OPEN_ELEVATOR5,
        WAIT_ELEVATOR_DELAY5,
        INTAKE5,
        PARK,
        IDLE
    }

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new SpinnerFirstPID(hardwareMap);
        cover = new Cover(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        freightSensor = new FreightSensor(hardwareMap);

        DriveConstants.setMaxVel(65);
        DriveConstants.setMaxAccel(65);

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(55),
                new RectangleMaskConstraint(45,-72,72,-45,
                        new TranslationalVelocityConstraint(10))));

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(55);

        startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        poseCollectCycle2 = new Pose2d(poseCollectX2 , poseCollectY2, Math.toRadians(poseCollectH2));

        MarkerCallback intakeForward =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                intake.intakeForward();
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback intakeBackword =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.intakeBackward(0.7);
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };


        MarkerCallback intakeStop =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.stop();
            }
        };

        MarkerCallback elevetorClose =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                closeElevator();
            }
        };

        drive.setPoseEstimate(startPoseRight);

        dip.getFreight();
        cover.closeCover();

        fixAngle = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseHelp, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL/3,
                        DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .lineToLinearHeading(new Pose2d(poseEntrance.getX(), poseEntrance.getY()-2, poseEntrance.getHeading()))
                .build();

        goToIntake1 = new TrajectorySequenceBuilder(fixAngle.end(), velConstraint, accelConstraint,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(new Pose2d(poseCollect.getX()+2, poseCollect.getY(), poseCollect.getHeading()))
                .build();

        goToHub2 = drive.trajectorySequenceBuilder(goToIntake1.end())
                .addTemporalMarker(intakeBackword)
                .lineToLinearHeading(poseEntrance)
                .build();

        goToIntake2 = new TrajectorySequenceBuilder(goToHub2.end(), velConstraint, accelConstraint,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(new Pose2d(poseCollect.getX()+4, poseCollect.getY(), poseCollect.getHeading()))
                .build();

        goToHub3 = drive.trajectorySequenceBuilder(goToIntake2.end())
                .addTemporalMarker(intakeBackword)
                .lineToLinearHeading(poseEntrance)
                .build();

        goToIntake3 = new TrajectorySequenceBuilder(goToHub3.end(), velConstraint, accelConstraint,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .splineTo(new Vector2d(poseCollect.getX()+8, poseCollect.getY()), Math.toRadians(10))
                .build();

        goToHub4 = drive.trajectorySequenceBuilder(goToIntake3.end())
                .addTemporalMarker(intakeBackword)
                .lineToLinearHeading(new Pose2d(poseEntrance.getX()-2, poseEntrance.getY(), poseEntrance.getHeading()))
                .build();

        goToIntake4 = new TrajectorySequenceBuilder(goToHub4.end(), velConstraint, accelConstraint,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .splineTo(new Vector2d(poseCollect.getX()+11, poseCollect.getY()), Math.toRadians(15))
                .build();

        goToHub5 = drive.trajectorySequenceBuilder(goToIntake4.end())
                .addTemporalMarker(intakeBackword)
                .lineToLinearHeading(new Pose2d(poseEntrance.getX()-3.5, poseEntrance.getY(), poseEntrance.getHeading()))
                .build();

        goToPark = new TrajectorySequenceBuilder(goToHub5.end(), velConstraint, accelConstraint,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .addTemporalMarker(elevetorClose)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .build();

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO_RED);

        waitForStart();

        if (isStopRequested()) return;

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        spinner.updateAuto();
        elevator.updateAuto();

        runningFor = new ElapsedTime();

        currentState = State.FIX_ANGLE;
        drive.followTrajectorySequenceAsync(fixAngle);

        while (opModeIsActive() && !isStopRequested()) {
            switchTrajs();

            hasFreight = freightSensor.hasFreight();

            // We update drive continuously in the background, regardless of state
            drive.update();

            elevator.updateAuto();
            spinner.updateAuto();
        }
    }

    void switchTrajs()
    {
        switch (currentState) {
            case FIX_ANGLE:
                if(!drive.isBusy())
                {
                    currentState = State.OPEN_ELEVATOR1;
                }
                break;
            case OPEN_ELEVATOR1:
                if(!drive.isBusy())
                {
                    intake.stop();
                    openElevator();
                    currentState = State.WAIT_ELEVATOR_DELAY1;
                }
                break;
            case WAIT_ELEVATOR_DELAY1:
                if(firstTime)
                {
                    offset = runningFor.seconds();
                    firstTime = false;
                }

                if((runningFor.seconds() - offset) >= elevatorDelay)
                {
                    firstTime = true;

                    closeElevator();

                    changeState(State.INTAKE1, goToIntake1);
                }
                break;
            case INTAKE1:
                if(hasFreight)
                {
                    drive.breakFollowing();
                    intake.intakeBackward();
                }

                changeState(State.OPEN_ELEVATOR2, goToHub2);
                break;
            case OPEN_ELEVATOR2:
                if(!drive.isBusy())
                {
                    intake.stop();
                    openElevator();
                    currentState = State.WAIT_ELEVATOR_DELAY2;
                }
                break;
            case WAIT_ELEVATOR_DELAY2:
                if(firstTime)
                {
                    offset = runningFor.seconds();
                    firstTime = false;
                }

                if((runningFor.seconds() - offset) >= elevatorDelay)
                {
                    firstTime = true;

                    closeElevator();

                    changeState(State.INTAKE2, goToIntake2);
                }
                break;
            case INTAKE2:
                if(hasFreight)
                {
                    drive.breakFollowing();
                    intake.intakeBackward();
                }

                changeState(State.OPEN_ELEVATOR3, goToHub3);
                break;
            case OPEN_ELEVATOR3:
                if(!drive.isBusy())
                {
                    intake.stop();
                    openElevator();
                    currentState = State.WAIT_ELEVATOR_DELAY3;
                }
                break;
            case WAIT_ELEVATOR_DELAY3:
                if(firstTime)
                {
                    offset = runningFor.seconds();
                    firstTime = false;
                }

                if((runningFor.seconds() - offset) >= elevatorDelay)
                {
                    firstTime = true;

                    closeElevator();

                    changeState(State.INTAKE3, goToIntake3);
                }
                break;
            case INTAKE3:
                if(hasFreight)
                {
                    drive.breakFollowing();
                    intake.intakeBackward();
                }

                changeState(State.OPEN_ELEVATOR4, goToHub4);
                break;
            case OPEN_ELEVATOR4:
                if(!drive.isBusy())
                {
                    intake.stop();
                    openElevator();
                    currentState = State.WAIT_ELEVATOR_DELAY4;
                }
                break;
            case WAIT_ELEVATOR_DELAY4:
                if(firstTime)
                {
                    offset = runningFor.seconds();
                    firstTime = false;
                }

                if((runningFor.seconds() - offset) >= elevatorDelay)
                {
                    firstTime = true;

                    closeElevator();

                    changeState(State.INTAKE4, goToIntake4);
                }
                break;
            case INTAKE4:
                if(hasFreight)
                {
                    drive.breakFollowing();
                    intake.intakeBackward();
                }

                changeState(State.OPEN_ELEVATOR5, goToHub5);
                break;
            case OPEN_ELEVATOR5:
                if(!drive.isBusy())
                {
                    intake.stop();
                    openElevator();
                    currentState = State.WAIT_ELEVATOR_DELAY5;
                }
                break;
            case WAIT_ELEVATOR_DELAY5:
                if(firstTime)
                {
                    offset = runningFor.seconds();
                    firstTime = false;
                }

                if((runningFor.seconds() - offset) >= elevatorDelay)
                {
                    firstTime = true;

                    closeElevator();

                    changeState(State.INTAKE5, goToPark);
                }
                break;
            case INTAKE5:
                if(hasFreight)
                {
                    intake.intakeBackward();
                }

                if(!drive.isBusy())
                {
                    currentState = State.PARK;
                }
                break;
            case PARK:
                requestOpModeStop();
                break;
        }
    }

    void changeState(State newState, TrajectorySequence traj)
    {
        if (!drive.isBusy()) {
            currentState = newState;
            drive.followTrajectorySequenceAsync(traj);
        }
    }

    void openElevator()
    {
        cover.openCover();

        powerElevator = powerElevatorFast;
        elevator.setPower(powerElevator);

        elevator.updateAuto();
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        spinner.updateAuto();


        dip.holdFreight();

        hand.level3();

        elevator.updateAuto();
        spinner.updateAuto();
    }

    void closeElevator()
    {
        dip.releaseFreight();

        powerElevator = powerSlowElevator;
        elevator.setPower(powerElevator);

        elevator.updateAuto();
        spinner.updateAuto();

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);

        elevator.updateAuto();
        spinner.updateAuto();

        hand.intake();

        cover.closeCover();
    }


}
