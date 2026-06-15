package org.firstinspires.ftc.teamcode.OpModes.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.TeleOpConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PathStorage;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;


@Autonomous(name = "BlueAuto")
public class BlueAuto extends LinearOpMode {
    private final Pose startPose = new Pose(20.454, 122.109, Math.toRadians(145));// Start Pose of our robot. This is against the goal facing AWAY

    private Follower follower;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path8;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path5;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;
    public PathChain Path12;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private boolean feedingBall = false;
    private static final double FEED_TIME = 3;
    private static final double PICKUP_SETTLE_TIME = 0.50;

    private FlyWheelSubsystem flyWheelSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TurretSubsystem turretSubsystem;
    private LimelightSubsystem limelightSubsystem;

    private HoodSubsystem hoodSubsystem;

    public void runOpMode() {
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap,gamepad2);
        intakeSubsystem  = new IntakeSubsystem(hardwareMap, gamepad2);
        turretSubsystem = new TurretSubsystem(hardwareMap);
        limelightSubsystem = new LimelightSubsystem(hardwareMap);
        limelightSubsystem.switchPipe(2);
        turretSubsystem.reset();
        hoodSubsystem = new HoodSubsystem(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.update();
        PathStorage.setPose(startPose);
        telemetry.addData("Pose state in Path Storage", PathStorage.getPose());
        telemetry.update();
        //After init
        waitForStart();
        //on start
        opmodeTimer.resetTimer();



        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();

            PathStorage.setPose(follower.getPose());
            telemetry.addData("Pose state in Path Storage", PathStorage.getPose());

            autonomousPathUpdate();
            intakeSubsystem.teleUpdate();
            flyWheelSubsystem.teleVelocity();



            hoodSubsystem.update(pose, TeleOpConstants.Turret.BLUE_TARGET_Y, TeleOpConstants.Turret.BLUE_TARGET_Y);


            double tx = 0;

            if(limelightSubsystem.hasTarget()){
                tx = limelightSubsystem.getTx();
            }

            turretSubsystem.update(follower.getPose(), TeleOpConstants.Turret.BLUE_TARGET_X, TeleOpConstants.Turret.BLUE_TARGET_Y,-tx);


            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intakeSubsystem.closeGate();

                startShooter();
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1:

                if(!follower.isBusy()) {

                    if(!feedingBall) {
                        intakeSubsystem.closeGate();
                    }

                    if(flyWheelSubsystem.atTargetVelocity()) {

                        if(!feedingBall) {

                            intakeSubsystem.openGate();
                            intakeSubsystem.autoPower(
                                    TeleOpConstants.Intake.TRANSFER_POWER
                            );

                            feedingBall = true;
                            pathTimer.resetTimer();
                        }

                        if(pathTimer.getElapsedTimeSeconds() > FEED_TIME) {

                            intakeSubsystem.closeGate();
                            feedingBall = false;

                            intakeSubsystem.autoPower(
                                    TeleOpConstants.Intake.INTAKE_POWER
                            );
                            follower.followPath(Path2, true);

                            setPathState(2);
                        }
                    }
                }

                break;

            case 2:

                if(!follower.isBusy()) {

                    if(pathTimer.getElapsedTimeSeconds() > PICKUP_SETTLE_TIME) {

                        stopIntake();
                        intakeSubsystem.autoPower(0);
                        startShooter();

                        follower.followPath(Path3, true);

                        setPathState(3);
                    }
                }

                break;

            case 3:

                if(!follower.isBusy()) {

                    if(flyWheelSubsystem.atTargetVelocity()) {

                        if(!feedingBall) {
                            intakeSubsystem.autoPower(0);
                            intakeSubsystem.openGate();
                            intakeSubsystem.autoPower(
                                    TeleOpConstants.Intake.TRANSFER_POWER
                            );

                            feedingBall = true;
                            pathTimer.resetTimer();
                        }

                        if(pathTimer.getElapsedTimeSeconds() > FEED_TIME) {

//                            intakeSubsystem.stop();
                            intakeSubsystem.closeGate();
                            feedingBall = false;

                            intakeSubsystem.autoPower(
                                    TeleOpConstants.Intake.INTAKE_POWER
                            );
                            follower.followPath(Path4, true);

                            setPathState(4);
                        }
                    }
                }

                break;


            case 4:

                if(!follower.isBusy()) {

                    stopIntake();
                    intakeSubsystem.autoPower(0);
                    startShooter();
                    intakeSubsystem.autoPower(0);
                    follower.followPath(Path5, true);

                    setPathState(5);
                }

                break;

            case 5:

                if(!follower.isBusy()) {

                    if(flyWheelSubsystem.atTargetVelocity()) {

                        if(!feedingBall) {
                            intakeSubsystem.autoPower(0);
                            intakeSubsystem.openGate();
                            intakeSubsystem.autoPower(
                                    TeleOpConstants.Intake.TRANSFER_POWER
                            );

                            feedingBall = true;
                            pathTimer.resetTimer();
                        }

                        if(pathTimer.getElapsedTimeSeconds() > FEED_TIME) {

                            intakeSubsystem.stop();
                            intakeSubsystem.closeGate();
                            feedingBall = false;



                            intakeSubsystem.autoPower(
                                    TeleOpConstants.Intake.INTAKE_POWER
                            );
                            follower.followPath(Path6, true);
                            setPathState(6);
                        }
                    }
                }

                break;


            case 6:

                if(!follower.isBusy()) {

                    stopIntake();
                    intakeSubsystem.autoPower(0);
                    startShooter();
                    intakeSubsystem.autoPower(0);

                    follower.followPath(Path7, true);

                    setPathState(7);
                }

                break;

            case 7:

                if(!follower.isBusy()) {

                    if(flyWheelSubsystem.atTargetVelocity()) {

                        if(!feedingBall) {

                            intakeSubsystem.openGate();
                            intakeSubsystem.autoPower(
                                    TeleOpConstants.Intake.TRANSFER_POWER
                            );

                            feedingBall = true;
                            pathTimer.resetTimer();
                        }

                        if(pathTimer.getElapsedTimeSeconds() > FEED_TIME) {

//                            intakeSubsystem.stop();
                            intakeSubsystem.closeGate();
                            stopShooter();

                            feedingBall = false;

                            follower.followPath(Path8, true);

                            setPathState(9);
                        }
                    }
                }

                break;

            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running and new paths */
                    setPathState(-1);
                }
                break;

        }


        // Update path storage to take current app
        PathStorage.setPose(follower.getPose());
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/



    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(22.160, 123.523),
                                new Pose(48.269, 94.115)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(48.269, 94.115),
                                new Pose(25.149, 94.041)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(25.149, 94.041),
                                new Pose(48.125, 94.048)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.125, 94.048),
                                new Pose(47.954, 93.231),
                                new Pose(49.440, 75.205),
                                new Pose(23.066, 77.711)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(23.066, 77.711),
                                new Pose(47.962, 93.907)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(47.962, 93.907),
                                new Pose(48.067, 54.732),
                                new Pose(51.496, 58.115),
                                new Pose(15.060, 58.070)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.060, 58.070),
                                new Pose(47.509, 94.021)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }



    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    private void startShooter() {

        flyWheelSubsystem.autoSetVelocity(
                TeleOpConstants.Flywheel.CLOSE_VEL
        );
    }

    private void stopShooter() {

        flyWheelSubsystem.stop();
    }

    private void startIntake() {

        intakeSubsystem.openGate();
        intakeSubsystem.autoPower(
                TeleOpConstants.Intake.TRANSFER_POWER
        );
    }

    private void stopIntake() {

        intakeSubsystem.stop();
    }

    private void shootRing() {

        if(flyWheelSubsystem.atTargetVelocity()) {

            intakeSubsystem.openGate();

            intakeSubsystem.autoPower(
                    TeleOpConstants.Intake.TRANSFER_POWER
            );
        }
    }
}