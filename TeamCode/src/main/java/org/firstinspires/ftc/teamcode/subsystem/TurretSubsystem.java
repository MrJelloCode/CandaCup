package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.TeleOpConstants;

public class TurretSubsystem {

    private final DcMotorEx turretMotor;

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
    }

    /* ================= AUTO AIM ================= */
    public void reset(){
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(
            Pose robotPose,
            double targetX,
            double targetY,
            double txCorrectionDeg
    ) {

        if (manualMode) return;

        updateTurretPosition();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double fieldTargetAngle = Math.atan2(dy, dx);

        double correctionDeg =
                Range.clip(
                        txCorrectionDeg,
                        -TeleOpConstants.Turret.MAX_TX_FOR_CORRECTION,
                        TeleOpConstants.Turret.MAX_TX_FOR_CORRECTION
                )
                        * TeleOpConstants.Turret.LL_CORRECTION_GAIN;

        double desiredTurretAngle =
                fieldTargetAngle
                        - robotHeading
                        + Math.toRadians(correctionDeg);

        while (desiredTurretAngle > Math.PI)
            desiredTurretAngle -= 2 * Math.PI;

        while (desiredTurretAngle < -Math.PI)
            desiredTurretAngle += 2 * Math.PI;

        turretTargetAngle = Range.clip(
                desiredTurretAngle,
                TeleOpConstants.Turret.MIN_ANGLE,
                TeleOpConstants.Turret.MAX_ANGLE
        );

        runPID();
    }
    /* ================= MANUAL ================= */

    public void manualControl(double stickInput) {

        manualMode = true;

        updateTurretPosition();

        double power = Range.clip(
                stickInput,
                -TeleOpConstants.Turret.MANUAL_POWER,
                TeleOpConstants.Turret.MANUAL_POWER
        );

        if (turretAngle >= TeleOpConstants.Turret.MAX_ANGLE && power > 0)
            power = 0;

        if (turretAngle <= TeleOpConstants.Turret.MIN_ANGLE && power < 0)
            power = 0;

        turretMotor.setPower(power);
    }

    public void disableManual() {
        manualMode = false;
    }

    /* ================= POSITION ================= */

    private void updateTurretPosition() {

        double ticks = turretMotor.getCurrentPosition();

        double motorRevs =
                ticks / TeleOpConstants.Turret.TICKS_PER_MOTOR_REV;

        double turretRevs =
                motorRevs / TeleOpConstants.Turret.GEAR_RATIO;

        turretAngle =
                turretRevs * (2.0 * Math.PI)
                        + TeleOpConstants.Turret.TURRET_OFFSET;
    }

    /* ================= PID ================= */

    private void runPID() {

        double error = turretTargetAngle - turretAngle;

        integral += error;

        double derivative = error - lastError;

        double output =
                TeleOpConstants.Turret.KP * error +
                        TeleOpConstants.Turret.KI * integral +
                        TeleOpConstants.Turret.KD * derivative;

        output = Range.clip(
                output,
                -TeleOpConstants.Turret.MAX_POWER,
                TeleOpConstants.Turret.MAX_POWER
        );

        /*
         * Hard-stop protection
         */

        if (turretAngle >= TeleOpConstants.Turret.MAX_ANGLE && output > 0) {
            output = 0;
        }

        if (turretAngle <= TeleOpConstants.Turret.MIN_ANGLE && output < 0) {
            output = 0;
        }

        turretMotor.setPower(output);

        lastError = error;
    }

    /* ================= TELEMETRY ================= */

    public void telemetry(Telemetry telemetry) {

        telemetry.addLine("------ TURRET ------");

        telemetry.addData(
                "Turret Angle Deg",
                Math.toDegrees(turretAngle)
        );

        telemetry.addData(
                "Target Angle Deg",
                Math.toDegrees(turretTargetAngle)
        );

        telemetry.addData(
                "Error Deg",
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

    /* ================= GETTERS =================Zz */

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getTargetAngle() {
        return turretTargetAngle;
    }
}