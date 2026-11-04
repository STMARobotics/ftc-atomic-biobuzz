package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.Limelight;

public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult result;
    private double lastTimestamp = -1;
    private boolean newFrame = false;

    // null when Limelight.ENABLED is false, then there's never a target
    public LimelightSubsystem(HardwareMap hardwareMap) {
        if (!Limelight.ENABLED) {
            limelight = null;
            return;
        }
        limelight = hardwareMap.get(Limelight3A.class, HardwareNames.LIMELIGHT);
        limelight.pipelineSwitch(Limelight.TAG_PIPELINE);
        limelight.start();
    }

    @Override
    public void periodic() {
        if (limelight == null) return;
        result = limelight.getLatestResult();

        double timestamp = result == null ? lastTimestamp : result.getTimestamp();
        newFrame = timestamp != lastTimestamp;
        lastTimestamp = timestamp;
    }

    /**
     * True for only the one loop where the camera gave us a frame we haven't seen yet
     */
    public boolean isNewFrame() {
        return newFrame;
    }

    public void setPipeline(int pipeline) {
        if (limelight != null) limelight.pipelineSwitch(pipeline);
    }

    public boolean hasTarget() {
        return result != null
                && result.isValid()
                && result.getStaleness() <= Limelight.MAX_STALENESS_MS;
    }

    /**
     * Degrees to the target, positive is right. 0 if there's no target
     */
    public double getXOffset() {
        return hasTarget() ? result.getTx() : 0.0;
    }

    public double getYOffset() {
        return hasTarget() ? result.getTy() : 0.0;
    }

    /**
     * Distance to the target in inches from ty, or NaN if there's no target
     */
    public double getDistance() {
        if (!hasTarget()) {
            return Double.NaN;
        }
        double angle = Math.toRadians(Limelight.CAMERA_ANGLE_DEG + result.getTy());
        double tangent = Math.tan(angle);
        if (tangent <= 0) {
            return Double.NaN;
        }
        return (Limelight.TAG_HEIGHT_IN - Limelight.CAMERA_HEIGHT_IN) / tangent;
    }

    public double getArea() {
        return hasTarget() ? result.getTa() : 0.0;
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Limelight has target", hasTarget());
        telemetry.addData("Limelight tx (deg)", "%.2f", getXOffset());
        telemetry.addData("Limelight ty (deg)", "%.2f", getYOffset());
    }
}
