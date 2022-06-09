package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import java.util.Arrays;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Auto - Right Red ASYNC", group = "Autonomous Red")
public class AutoRightRedAsync extends LinearOpMode {

    public static double startPoseRightX = 13;
    public static double startPoseRightY = -54.28;
    public static double startPoseRightH = 90;

    public static double poseFixAngleX = 7;
    public static double poseFixAngleY = -50;
    public static double poseFixAngleH = 180;
    public static double poseHubX = 12.5;
    public static double poseHubY = -58;
    public static double poseHubH = 180;
    public static double poseIntakeX = 60;
    public static double poseIntakeY = -58;
    public static double poseIntakeH = 180;
    public static double poseIntakeFifteenX = 70;
    public static double poseIntakeFifteenY = -57;
    public static double poseIntakeFifteenH = 15;
    public static double poseIntakeThirtyH = 30;

    public static double GO_PARK_AT = 28;
    public static double powerSlowElevator = .7, powerElevator = 1, powerElevatorFast = 1;
    public static double elevatorDelay = 1;

    int wentIntakeXTimes = 0;
    boolean hasFreight = false;
    double offset = 0;
    boolean firstTime = true;

    SampleMecanumDrive drive;
    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    dip dip;
    intake intake;
    Cover cover;
    ElapsedTime runningFor;

    TrajectorySequence fixAngle, goToHub, straightLineIntake, fifteenDegreeIntake, thirtyDegreeIntake, park;
    Pose2d startPoseRight, poseFixAngle, poseHub, poseGoToIntake, poseGoToIntakeFifteen, poseGoToIntakeThirty;

    enum State
    {
        FIX_ANGLE, // fix angle from start angle
        INTAKE, // go to intake
        IDLE, // nothing
        LEAVE_EVERYTHING_AND_PARK,
        OPEN_ELEVATOR,
        WAIT_ELEVATOR_DELAY
    }

    enum intakePathType
    {
        STRAIGHT_PATH,
        FIFTEEN_DEGREE_PATH,
        THIRTY_DEGREE_PATH
    }

    State currentState = State.IDLE;
    intakePathType intakeType = intakePathType.STRAIGHT_PATH;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new SpinnerFirstPID(hardwareMap);
        hand = new hand(hardwareMap);
        intake = new intake(hardwareMap);
        dip = new dip(hardwareMap);
        cover = new Cover(hardwareMap);

        startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        poseFixAngle = new Pose2d(poseFixAngleX, poseFixAngleY, Math.toRadians(poseFixAngleH));
        poseHub = new Pose2d(poseHubX, poseHubY, Math.toRadians(poseHubH));
        poseGoToIntake = new Pose2d(poseIntakeX, poseIntakeY, Math.toRadians(poseIntakeH));
        poseGoToIntakeFifteen = new Pose2d(poseIntakeFifteenX, poseIntakeFifteenY, Math.toRadians(poseIntakeFifteenH));
        poseGoToIntakeThirty = new Pose2d(poseIntakeFifteenX, poseIntakeFifteenY, Math.toRadians(poseIntakeThirtyH));

        drive.setPoseEstimate(startPoseRight);

        MarkerCallback intakeForward =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                intake.intakeForward();
            }
        };

        MarkerCallback intakeBackword =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.intakeBackward();
            }
        };

        MarkerCallback intakeStop =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.stop();
            }
        };


        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(80),
                new RectangleMaskConstraint(35,-72,72,-35,
                        new TranslationalVelocityConstraint(10))));

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(60);


        // Let's define our trajectories
        fixAngle = drive.trajectorySequenceBuilder(startPoseRight)
                .lineToLinearHeading(poseFixAngle)
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        goToHub = drive.trajectorySequenceBuilder(fixAngle.end())
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseHub)
                .build();

        straightLineIntake = new TrajectorySequenceBuilder(goToHub.end(), velConstraint, accelConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseGoToIntake)
                .strafeLeft(2)
                .build();

        fifteenDegreeIntake = new TrajectorySequenceBuilder(goToHub.end(), velConstraint, accelConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseGoToIntake)
                .splineTo(new Vector2d(poseGoToIntakeFifteen.getX(), poseGoToIntakeFifteen.getY()), poseGoToIntakeFifteen.getHeading())
                .lineToSplineHeading(poseGoToIntake)
                .strafeLeft(2)
                .build();

        thirtyDegreeIntake = new TrajectorySequenceBuilder(goToHub.end(), velConstraint, accelConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseGoToIntake)
                .splineTo(new Vector2d(poseGoToIntakeThirty.getX(), poseGoToIntakeThirty.getY()), poseGoToIntakeThirty.getHeading())
                .lineToSplineHeading(poseGoToIntake)
                .strafeLeft(2)
                .build();

        park = drive.trajectorySequenceBuilder(goToHub.end())
                .addTemporalMarker(intakeStop)
                .lineToSplineHeading(poseGoToIntake)
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

            if(runningFor.seconds() >= GO_PARK_AT)
            {
                currentState = State.LEAVE_EVERYTHING_AND_PARK;
            }

            // We update drive continuously in the background, regardless of state
            drive.update();

            elevator.updateAuto();
            spinner.updateAuto();
//
//            // Read pose
//            Pose2d poseEstimate = drive.getPoseEstimate();
//
//            // Continually write pose to `PoseStorage`
//            PoseStorage.currentPose = poseEstimate;
        }
    }

    void changeState(State newState, TrajectorySequence traj)
    {
        if (!drive.isBusy()) {
            currentState = newState;
            drive.followTrajectorySequenceAsync(traj);
        }
    }

    void changeStateIntake(State newState, TrajectorySequence traj)
    {
        if (!drive.isBusy()) {
            currentState = newState;
            drive.followTrajectorySequenceAsync(traj);

            wentIntakeXTimes++;

            switch (wentIntakeXTimes)
            {
                case 1:
                    intakeType = intakePathType.STRAIGHT_PATH;
                    break;
                case 2:
                    intakeType = intakePathType.FIFTEEN_DEGREE_PATH;
                    break;
                case 3:
                    intakeType = intakePathType.THIRTY_DEGREE_PATH;
                    wentIntakeXTimes = 0;
                    break;
            }
        }
    }

    void switchTrajs()
    {
        switch (currentState) {
            case FIX_ANGLE:
                changeState(State.OPEN_ELEVATOR, goToHub);
                break;
            case OPEN_ELEVATOR:
                if(!drive.isBusy())
                {
                    intake.stop();
                    openElevator();
                    currentState = State.WAIT_ELEVATOR_DELAY;
                }
                break;
            case WAIT_ELEVATOR_DELAY:
                if(firstTime)
                {
                    offset = runningFor.seconds();
                    firstTime = false;
                }

                if((runningFor.seconds() - offset) >= elevatorDelay)
                {
                    firstTime = true;

                    closeElevator();

                    switch (intakeType)
                    {
                        case STRAIGHT_PATH:
                            changeStateIntake(State.INTAKE, straightLineIntake);
                            break;
                        case FIFTEEN_DEGREE_PATH:
                            changeStateIntake(State.INTAKE, fifteenDegreeIntake);
                            break;
                        case THIRTY_DEGREE_PATH:
                            changeStateIntake(State.INTAKE, thirtyDegreeIntake);
                            break;
                    }
                }
                break;
            case INTAKE:
                if(hasFreight)
                {
                    drive.breakFollowing();
                }

                if (!drive.isBusy()) {
                    changeState(State.OPEN_ELEVATOR, goToHub);
                }
                break;
            case LEAVE_EVERYTHING_AND_PARK:
                if (drive.isBusy())
                {
                    drive.breakFollowing();

                    Pose2d currentPose = drive.getPoseEstimate();

                    TrajectorySequence parkNow = drive.trajectorySequenceBuilder(currentPose)
                            .lineToSplineHeading(poseGoToIntake)
                            .build();

                    closeElevator();
                    intake.stop();

                    drive.followTrajectorySequenceAsync(parkNow);
                }
                else
                {
                    changeState(State.IDLE, park);
                }
                break;
            case IDLE:
                requestOpModeStop();
                break;
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

        dip.releaseFreight();

        hand.intake();

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);

        elevator.updateAuto();
        spinner.updateAuto();

        cover.closeCover();
    }
}
