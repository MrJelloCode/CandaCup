package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.TeleOpConstants;

public class TurretSubsystem {

    private DcMotorEx turretMotor;

    private double turretAngle = 0;
    private double turretTargetAngle = 0;

    private double integral = 0;
    private double lastError = 0;

    private boolean manualMode = false;

    public TurretSubsystem(HardwareMap hardwareMap) {

        turretMotor = hardwareMap.get(
                DcMotorEx.class,
                TeleOpConstants.Turret.TURRET_MOTOR_NAME
        );

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ========= AUTO AIM ========= */

    public void update(Pose robotPose, double targetX, double targetY) {

        if(manualMode) return;

        updateTurretPosition();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double fieldTargetAngle = Math.atan2(dy, dx);

        double rawTarget = fieldTargetAngle - robotHeading;

        turretTargetAngle = wrapToSafeRange(rawTarget);

        runPID();
    }

    /* ========= MANUAL CONTROL ========= */

    public void manualControl(double stickInput) {

        manualMode = true;

        double power = Range.clip(
                stickInput,
                -TeleOpConstants.Turret.MANUAL_POWER,
                TeleOpConstants.Turret.MANUAL_POWER
        );

        turretMotor.setPower(power);
    }

    public void disableManual() {
        manualMode = false;
    }


    private void updateTurretPosition() {

        double ticks = turretMotor.getCurrentPosition();

        double motorRevs = ticks / TeleOpConstants.Turret.TICKS_PER_MOTOR_REV;

        double turretRevs = motorRevs / TeleOpConstants.Turret.GEAR_RATIO;

        turretAngle = turretRevs * 2 * Math.PI + TeleOpConstants.Turret.TURRET_OFFSET;

        while (turretAngle > Math.PI)
            turretAngle -= 2 * Math.PI;

        while (turretAngle < -Math.PI)
            turretAngle += 2 * Math.PI;
    }

    /* =========  WRAP ========= */

    private double wrapToSafeRange(double angle) {

        while(angle > Math.PI) angle -= 2 * Math.PI;
        while(angle < -Math.PI) angle += 2 * Math.PI;

        if(angle > TeleOpConstants.Turret.MAX_ANGLE)
            angle -= 2 * Math.PI;

        if(angle < TeleOpConstants.Turret.MIN_ANGLE)
            angle += 2 * Math.PI;

        return angle;
    }

    /* ========= PID CONTROLLER ========= */

    private void runPID() {

        double error = turretTargetAngle - turretAngle;

        while(error > Math.PI)
            error -= 2 * Math.PI;

        while(error < -Math.PI)
            error += 2 * Math.PI;

        integral += error;
        double derivative = error - lastError;

        double output = TeleOpConstants.Turret.KP * error +
                        TeleOpConstants.Turret.KI * integral +
                        TeleOpConstants.Turret.KD * derivative;

        output = Range.clip(
                output,
                -TeleOpConstants.Turret.MAX_POWER,
                TeleOpConstants.Turret.MAX_POWER
        );

        turretMotor.setPower(output);

        lastError = error;
    }

    /* ========= TELEMETRY ========= */

    public void telemetry(Telemetry telemetry) {

        telemetry.addLine("------ TURRET ------");

        telemetry.addData(
                "Turret Angle (deg)",
                Math.toDegrees(turretAngle)
        );

        telemetry.addData(
                "Target Angle (deg)",
                Math.toDegrees(turretTargetAngle)
        );

        telemetry.addData(
                "Error (deg)",
                Math.toDegrees(turretTargetAngle - turretAngle)
        );

        telemetry.addData(
                "Encoder",
                turretMotor.getCurrentPosition()
        );

        telemetry.addData(
                "Manual Mode",
                manualMode
        );
    }

    /* ========= GETTERS ========= */

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getTargetAngle() {
        return turretTargetAngle;
    }
}