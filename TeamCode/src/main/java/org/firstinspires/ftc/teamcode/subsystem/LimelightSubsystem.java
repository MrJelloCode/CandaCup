package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.TeleOpConstants;

/**
 * Streamlined Limelight-based shooter helper
 * Focused on geometry + hood angle + flywheel size
 */
public class LimelightSubsystem {

    private Limelight3A limelight;
    private double tx, ty, ta;
    private boolean hasTarget, updating = false;


    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, TeleOpConstants.Limelight.LIMELIGHT_NAME);
        limelight.pipelineSwitch(TeleOpConstants.Limelight.DEFAULT_PIPELINE);
        limelight.setPollRateHz(TeleOpConstants.Limelight.LIMELIGHT_POLL_RATE_HZ);
        limelight.start();
    }

    public void switchPipe(int index) {
        limelight.pipelineSwitch(index);
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            hasTarget = true;
        } else {
            tx = ty = ta = 0;
            hasTarget = false;
        }
        updating = true;
    }

    public boolean hasTarget() { return hasTarget; }
    public boolean isUpdating() { return updating; }

    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }

    /* =====================
       DISTANCE ESTIMATION
       ===================== */

    public double getDistanceInches() {
        double angleRad = Math.toRadians(TeleOpConstants.Limelight.LIMELIGHT_PITCH_DEGREES + ty);
        if (Math.abs(Math.tan(angleRad)) < 0.01) return 0;

        double distance = (TeleOpConstants.Limelight.TARGET_HEIGHT_INCHES - TeleOpConstants.Limelight.LIMELIGHT_HEIGHT_INCHES)
                / Math.tan(angleRad);

        return distance + TeleOpConstants.Limelight.DISTANCE_OFFSET_INCHES;
    }

    /* =====================
       BALLISTIC SOLVER
       ===================== */

    private double calculateExitVelocityIPS() {
        double d = getDistanceInches();
        if (d <= 0) return 0;

        double theta = Math.toRadians(TeleOpConstants.Flywheel.HOOD_ANGLE_DEGREES);
        double deltaH = TeleOpConstants.Limelight.TARGET_HEIGHT_INCHES - TeleOpConstants.Flywheel.FLYWHEEL_HEIGHT_INCHES;

        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double denominator = 2 * cos * cos * (d * tan - deltaH);
        if (denominator <= 0) return 0;

        return Math.sqrt((TeleOpConstants.Limelight.GRAVITY * d * d) / denominator);
    }

    /* =====================
       RPM CONVERSION
       ===================== */

    public double getRequiredShooterRPM() {
        double v = calculateExitVelocityIPS();
        if (v <= 0) return 0;

        double wheelCircumference = Math.PI * TeleOpConstants.Flywheel.FLYWHEEL_DIAMETER_INCHES;
        double baseRPM = (v / wheelCircumference) * 60.0;

        return baseRPM + TeleOpConstants.Limelight.RPM_OFFSET;
    }
}