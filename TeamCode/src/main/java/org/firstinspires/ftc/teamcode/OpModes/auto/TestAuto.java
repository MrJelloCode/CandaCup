package org.firstinspires.ftc.teamcode.OpModes.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "testAuto")
public class TestAuto extends LinearOpMode {
    private final Pose startPose = new Pose(20.454, 122.109, Math.toRadians(143));// Start Pose of our robot. This is against the goal facing AWAY
    private final Pose scorePose = new Pose(54.112, 84.087); // Scoring Pose of our robot.
    private final Pose drinkFromFountain = new Pose(11.803, 61.585, Math.toRadians(146)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(12, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(12, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose endPose = new Pose(60, 105); // Final Pose of our robot, off the starting line
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
    public PathChain line10;
    public PathChain Path11;
    public PathChain Path12;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();
        //on start
        opmodeTimer.resetTimer();
        setPathState(0);

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();


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
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(20.454, 122.109),
                                new Pose(53.924, 84.219)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(143))
                .build();
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.924, 84.219),
                                new Pose(44.275, 56.333),
                                new Pose(32.187, 60.079),
                                new Pose(8.886, 59.377)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.886, 59.377),
                                new Pose(54.286, 84.247)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierCurve (curved line). */
        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(54.286, 84.247),
                                new Pose(26.475, 51.840),
                                new Pose(11.773, 61.604)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.773, 61.604),
                                new Pose(54.112, 84.087)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(54.112, 84.087),
                                new Pose(26.888, 51.781),
                                new Pose(11.712, 61.581)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.712, 61.581),
                                new Pose(54.091, 84.135)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(54.091, 84.135),
                                new Pose(14.381, 83.797)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.381, 83.797),
                                new Pose(54.171, 84.175)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        line10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(54.171, 84.175),
                                new Pose(41.819, 35.613)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(41.819, 35.613),
                                new Pose(9.394, 35.286)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.394, 35.286),
                                new Pose(54.147, 84.383)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}