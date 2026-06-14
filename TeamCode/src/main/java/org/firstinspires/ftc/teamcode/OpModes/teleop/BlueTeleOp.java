package org.firstinspires.ftc.teamcode.OpModes.teleop;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.TeleOpConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PathStorage;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

@Configurable
@TeleOp(name="BlueTeleOp")
public class BlueTeleOp extends OpMode {

    private LimelightSubsystem limelightSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;
    private FlyWheelSubsystem flyWheelSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TurretSubsystem turretSubsystem;
    private static TelemetryManager panelsTelemetry;
    private HoodSubsystem hoodSubsystem;
    private boolean autoAimEnabled = true;
    private boolean lastY = false;
//    private RevBlinkinLedDriver blinkin;

    private Follower follower;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(TeleOpConstants.Turret.RED_START_X,TeleOpConstants.Turret.RED_START_Y, TeleOpConstants.Turret.RED_START_HEADING) == null ? new Pose() : new Pose(TeleOpConstants.Turret.RED_START_X,TeleOpConstants.Turret.RED_START_Y, TeleOpConstants.Turret.RED_START_HEADING));

        follower.update();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        limelightSubsystem = new LimelightSubsystem(hardwareMap);
        limelightSubsystem.switchPipe(1);
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap,gamepad1);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap,gamepad2);
        intakeSubsystem  = new IntakeSubsystem(hardwareMap, gamepad2);
        turretSubsystem = new TurretSubsystem(hardwareMap);
        hoodSubsystem = new HoodSubsystem(hardwareMap);
//        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        follower = Constants.createFollower(hardwareMap);

//        follower.setStartingPose(new Pose(
//                TeleOpConstants.Turret.RED_START_X,
//                TeleOpConstants.Turret.RED_START_Y,
//                Math.toRadians(TeleOpConstants.Turret.RED_START_HEADING)
//
//
//        ));


        follower.setStartingPose(PathStorage.getPose()); // Restore auto pose ONCE
        follower.update();
        telemetry.addData("PATH CURRENT", PathStorage.getPose());
        telemetry.update();
//        follower.update();
    }

    @Override
    public void loop() {

        limelightSubsystem.update();
        drivetrainSubsystem.robotCentricDrive();
        intakeSubsystem.teleUpdate();
        flyWheelSubsystem.teleVelocity();
        flyWheelSubsystem.graphTelemetry(panelsTelemetry);






//        if(flyWheelSubsystem.atTargetVelocity()){
//            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//        }else{
//            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
//        }


        Pose pose = follower.getPose();

        if(gamepad1.start){
            follower.setPose(new Pose(10,10,Math.toRadians(90)));
        }


        hoodSubsystem.update(pose, TeleOpConstants.Turret.BLUE_TARGET_X, TeleOpConstants.Turret.BLUE_TARGET_Y);

        /* ================= AUTO AIM TOGGLE ================= */

        if(gamepad2.y && !lastY){
            autoAimEnabled = !autoAimEnabled;
        }

        lastY = gamepad2.y;

        /* ================= TURRET CONTROL ================= */

        if(autoAimEnabled){

            turretSubsystem.disableManual();


            double tx = 0;

            if(limelightSubsystem.hasTarget()){
                tx = limelightSubsystem.getTx();
            }

            turretSubsystem.update(
                    pose,
                    TeleOpConstants.Turret.BLUE_TARGET_X,
                    TeleOpConstants.Turret.BLUE_TARGET_Y,
                    tx
            );
        }
        else{

            if(Math.abs(gamepad2.right_stick_x) > 0.1){
                turretSubsystem.manualControl(
                        gamepad2.right_stick_x
                );
            }
            else{
                turretSubsystem.manualControl(0);
            }
        }
//        if(gamepad2.dpad_up){
//            hoodSubsystem.adjust(0.02);
//        }
//
//        if(gamepad2.dpad_down){
//            hoodSubsystem.adjust(-0.02);
//        }




        follower.update();



        //telemtry
        turretSubsystem.telemetry(telemetry);
        hoodSubsystem.telemetry(telemetry);
        telemetry.addData("Turret Auto Aim", autoAimEnabled ? "ON" : "OFF");
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        telemetry.update();



    }
}