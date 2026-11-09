package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Devices;

public class LimelightSubsystem extends SubsystemBase {

    // manual 9.9: tags 30 to 45, 4 on each of the 4 cells
    private static final int FIRST_TAG_ID = 30;
    private static final int TAGS_PER_CELL = 4;
    private static final int CELLS = 4;
    // where each tag is on its sticker, inches from the center (manual figure 9-15), in ID order
    private static final double[] TAG_POSITIONS = {-6.5, -2.75, 2.75, 6.5};
    // another cell has to be this much higher in the frame to switch to it, so noise doesn't flip the aim
    private static final double SWITCH_CELL_DEG = 3;

    private final Limelight3A limelight;
    private LLResult result;
    private Alliance alliance = null; // null counts every Hive's tags
    private boolean cellInView = false;
    private double cellTx = 0, cellTy = 0, cellArea = 0;
    private int currentCell = -1;
    // degrees per inch along the sticker, what a lone tag needs to find the center
    private final double[] slopeTx = new double[CELLS];
    private final double[] slopeTy = new double[CELLS];
    private double lastTimestamp = -1;
    private boolean newFrame = false;

    // when the tag last showed up / went away, for the seesaw check
    private boolean hadTarget = false;
    private long targetChangedNanos = System.nanoTime();

    // null when Limelight.ENABLED is false or it's missing from the config, then there's never a target
    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = Limelight.ENABLED
                ? Devices.optional(hardwareMap, Limelight3A.class, HardwareNames.LIMELIGHT)
                : null;
        if (limelight == null) return;
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
        pickCell();

        boolean has = hasTarget();
        if (has != hadTarget) {
            hadTarget = has;
            targetChangedNanos = System.nanoTime();
        }
    }

    /**
     * Each cell has 4 tags in a row, only ours count. We want the middle of the cell, and that has to hold
     * up when some of the tags are out of frame (leading a shot pushes them to the edge), so an average
     * won't do: it jumps every time one drops out. A line through the tags we can see gives the middle
     * from any of them. With a lone tag we use the slope from the last time we saw two.
     * With more than one cell in view it's the highest one in the frame: that's the raised one, the one you
     * can score in. Not the biggest, from beside the Hive the lowered one is closer
     */
    private void pickCell() {
        cellInView = false;
        if (result == null || !result.isValid()) return;
        int[] n = new int[CELLS];
        double[] area = new double[CELLS], sumP = new double[CELLS], sumPP = new double[CELLS];
        double[] sumTx = new double[CELLS], sumPTx = new double[CELLS], sumTy = new double[CELLS], sumPTy = new double[CELLS];
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            int cell = (id - FIRST_TAG_ID) / TAGS_PER_CELL;
            if (id < FIRST_TAG_ID || cell >= CELLS) continue; // not a tag on a cell
            if (alliance != null && !alliance.ownsTag(id)) continue;
            double p = TAG_POSITIONS[(id - FIRST_TAG_ID) % TAGS_PER_CELL];
            n[cell]++;
            area[cell] += tag.getTargetArea();
            sumP[cell] += p;
            sumPP[cell] += p * p;
            sumTx[cell] += tag.getTargetXDegrees();
            sumPTx[cell] += p * tag.getTargetXDegrees();
            sumTy[cell] += tag.getTargetYDegrees();
            sumPTy[cell] += p * tag.getTargetYDegrees();
        }

        int best = -1;
        for (int cell = 0; cell < CELLS; cell++) {
            if (n[cell] > 0 && (best < 0 || sumTy[cell] / n[cell] > sumTy[best] / n[best])) best = cell;
        }
        if (best < 0) return;
        boolean keep = currentCell >= 0 && n[currentCell] > 0
                && sumTy[best] / n[best] < sumTy[currentCell] / n[currentCell] + SWITCH_CELL_DEG;
        int cell = keep ? currentCell : best;
        currentCell = cell;

        double spread = n[cell] * sumPP[cell] - sumP[cell] * sumP[cell];
        if (n[cell] >= 2 && spread > 0) {
            slopeTx[cell] = (n[cell] * sumPTx[cell] - sumP[cell] * sumTx[cell]) / spread;
            slopeTy[cell] = (n[cell] * sumPTy[cell] - sumP[cell] * sumTy[cell]) / spread;
        }
        cellTx = (sumTx[cell] - slopeTx[cell] * sumP[cell]) / n[cell];
        cellTy = (sumTy[cell] - slopeTy[cell] * sumP[cell]) / n[cell];
        cellArea = area[cell];
        cellInView = true;
    }

    /**
     * Only tags on this alliance's cells count from now on, null for all of them
     */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    /**
     * How old the latest frame is, from when the camera took it to now
     */
    public double getLatencyMs() {
        if (result == null) return 0;
        return result.getCaptureLatency() + result.getTargetingLatency() + result.getStaleness();
    }

    /**
     * True for only the one loop where the camera gave us a frame we haven't seen yet
     */
    public boolean isNewFrame() {
        return newFrame;
    }

    /**
     * False when it's turned off or missing from the config, then nothing should wait on a target
     */
    public boolean isPresent() {
        return limelight != null;
    }

    /**
     * Whether the camera is still sending frames, tag or no tag. No point looking for a tag with a dead camera
     */
    public boolean isAlive() {
        return result != null && result.getStaleness() <= Limelight.ALIVE_STALENESS_MS;
    }

    public void setPipeline(int pipeline) {
        if (limelight != null) limelight.pipelineSwitch(pipeline);
    }

    public boolean hasTarget() {
        return cellInView
                && result != null
                && result.getStaleness() <= Limelight.MAX_STALENESS_MS;
    }

    /**
     * How long the tag has been in view without dropping out, 0 if we can't see it
     */
    public double msTargetVisible() {
        return hadTarget ? (System.nanoTime() - targetChangedNanos) / 1e6 : 0;
    }

    /**
     * How long the tag has been out of view, 0 if we can see it
     */
    public double msTargetHidden() {
        return hadTarget ? 0 : (System.nanoTime() - targetChangedNanos) / 1e6;
    }

    /**
     * The tag is hidden when the seesaw is tipped toward us, so seeing it steadily means we can score.
     * Only trust this once we're parked facing the target.
     */
    public boolean seesawScorable() {
        return msTargetVisible() >= Limelight.SEESAW_CONFIRM_MS;
    }

    /**
     * Degrees to the target, positive is right. 0 if there's no target
     */
    public double getXOffset() {
        return hasTarget() ? cellTx : 0.0;
    }

    public double getYOffset() {
        return hasTarget() ? cellTy : 0.0;
    }

    /**
     * Distance to the target in inches from ty, or NaN if there's no target
     */
    public double getDistance() {
        if (!hasTarget()) {
            return Double.NaN;
        }
        double angle = Math.toRadians(Limelight.CAMERA_ANGLE_DEG + cellTy);
        double tangent = Math.tan(angle);
        if (tangent <= 0) {
            return Double.NaN;
        }
        return (Limelight.TAG_HEIGHT_IN - Limelight.CAMERA_HEIGHT_IN) / tangent;
    }

    public double getArea() {
        return hasTarget() ? cellArea : 0.0;
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    public void telemetrize(Telemetry telemetry) {
        if (limelight == null) {
            telemetry.addData("Limelight", "off");
            return;
        }
        telemetry.addData("Limelight has target", hasTarget());
        telemetry.addData("Tags from", alliance != null ? alliance : "ANY HIVE, pick an alliance in INIT");
        telemetry.addData("Tracking tags", hasTarget()
                ? (FIRST_TAG_ID + currentCell * TAGS_PER_CELL) + "-" + (FIRST_TAG_ID + currentCell * TAGS_PER_CELL + TAGS_PER_CELL - 1)
                : "none");
        telemetry.addData("Seesaw scorable", seesawScorable());
        telemetry.addData("Limelight tx (deg)", "%.2f", getXOffset());
        telemetry.addData("Limelight ty (deg)", "%.2f", getYOffset());
    }
}
