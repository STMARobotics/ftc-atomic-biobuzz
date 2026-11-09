package sim;

import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;

import java.util.ArrayList;
import java.util.List;

/**
 * Works out Shooter.SHOT_TABLE by firing real shots through the sim's physics: every mix of distance,
 * rpm and hood, from a few angles around the cell, pollen and nectar. Prints .add(...) lines to paste in.
 * Run it again whenever the shot physics change:  gradlew -p sim shotTable
 *
 * The table is only as true as the sim's guesses (mostly BALL_EXIT_FRACTION and the hood mapping).
 * Real shots in ShotTuner beat it, this is a starting point that actually hits.
 */
final class ShotTableSweep {

    private static final double[] DISTANCES = {24, 30, 36, 42, 48, 54, 60, 66, 72, 80, 90};
    private static final int RPM_MIN = 2400, RPM_MAX = 3800, RPM_STEP = 100;
    private static final int HOOD_STEPS = 10;
    // closest to straight on first, walls rule some out. Negative is toward the middle of the field
    private static final double[] ANGLES_DEG = {0, 15, -15, 30, -30, 45, -45, 57, -57, 65, -65};
    private static final int MAX_ANGLES = 4;

    // the raised red cell's tag and the room around it (a robot can't go past the walls)
    private static final double TAG_X = 59.26, TAG_Y = 59.11;
    private static final double WALL_MARGIN = 10;
    // flight time counts to here on the way down, about the middle of the cell's opening
    private static final double FLIGHT_END_Z = 59;

    private final SimWorld world = new SimWorld();
    private final SimPhysics physics = world.physics();

    private double lastFlight = 0;

    private ShotTableSweep() {}

    /** Robot spots at this camera-to-tag distance, facing the tag with the turret forward. {x, y, heading} */
    private List<double[]> spots(double distance) {
        List<double[]> all = new ArrayList<>();
        double fromTurret = distance + SimPhysics.CAMERA_FORWARD;
        for (double angle : ANGLES_DEG) {
            double a = Math.toRadians(angle);
            double dx = Math.sin(a), dy = Math.cos(a);
            double facing = Math.atan2(dy, dx);
            // the robot center sits behind the turret axis
            double x = TAG_X - fromTurret * dx - SimPhysics.TURRET_AXIS_X * Math.cos(facing);
            double y = TAG_Y - fromTurret * dy - SimPhysics.TURRET_AXIS_X * Math.sin(facing);
            if (x < WALL_MARGIN || x > 144 - WALL_MARGIN || y < WALL_MARGIN) continue;
            // the robot has to actually fit there: something in the way (a flower) shoves it off the spot
            physics.collisions.resetPieces();
            physics.placeRobot(x, y, facing);
            settle(0.15);
            if (Math.hypot(physics.x - x, physics.y - y) > 0.5) continue;
            all.add(new double[]{x, y, facing, angle});
        }
        return all.size() > MAX_ANGLES ? all.subList(0, MAX_ANGLES) : all;
    }

    private void settle(double seconds) {
        int steps = (int) (seconds / SimCollisions.STEP_SEC);
        for (int i = 0; i < steps; i++) physics.step(SimCollisions.STEP_SEC);
    }

    /** One shot from a fresh field, true if it ends up resting in the cell */
    private boolean fire(int type, double[] spot, double rpm, double hood) {
        physics.collisions.resetPieces();
        physics.placeRobot(spot[0], spot[1], spot[2]);
        settle(0.2);
        SimCollisions.Hive red = physics.collisions.redHive;
        boolean pollen = type == SimCollisions.POLLEN;
        world.servo(pollen ? HardwareNames.POLLEN_HOOD_SERVO : HardwareNames.NECTAR_HOOD_SERVO).physical = hood;
        physics.flywheelRpm = rpm;
        physics.launch(type);

        double t = 0, apex = 0, flight = 0;
        double restTimer = 0, lastX = 0, lastY = 0, lastZ = 0;
        while (t < 4) {
            settle(0.02);
            t += 0.02;
            double[] p = physics.collisions.pieces();
            if (p.length < 4) return false;
            double x = p[p.length - 3], y = p[p.length - 2], z = p[p.length - 1];
            apex = Math.max(apex, z);
            if (flight == 0 && apex > FLIGHT_END_Z && z <= FLIGHT_END_Z) flight = t;
            if (t > 0.3 && z < 12) return false; // on its way to the floor
            // this ball, sitting still, inside the cell
            boolean still = Math.abs(x - lastX) + Math.abs(y - lastY) + Math.abs(z - lastZ) < 0.15;
            lastX = x;
            lastY = y;
            lastZ = z;
            if (still && red.holds(x * 0.0254, y * 0.0254, z * 0.0254)) restTimer += 0.02;
            else restTimer = 0;
            if (restTimer > 0.15) {
                lastFlight = flight;
                return true;
            }
            if (t > 0.5 && apex < 45 && z < apex - 3) return false; // nowhere near high enough
        }
        return false;
    }

    private static final class Result {
        final int[][] count; // [rpm][hood] how many spots went in
        final boolean[][] good; // [rpm][hood] as many as anything managed
        final double[][] flight;

        Result(int rpms, int hoods) {
            count = new int[rpms][hoods];
            good = new boolean[rpms][hoods];
            flight = new double[rpms][hoods];
        }

        int best(int rpm) {
            int best = 0;
            for (int c : count[rpm]) best = Math.max(best, c);
            return best;
        }

        void markGood(int needed) {
            for (int i = 0; i < count.length; i++) {
                for (int h = 0; h < count[i].length; h++) good[i][h] = needed > 0 && count[i][h] >= needed;
            }
        }
    }

    /** How many of the spots go in, for every rpm and hood */
    private Result sweep(int type, List<double[]> spots) {
        int rpms = (RPM_MAX - RPM_MIN) / RPM_STEP + 1;
        Result r = new Result(rpms, HOOD_STEPS + 1);
        for (int i = 0; i < rpms; i++) {
            for (int h = 0; h <= HOOD_STEPS; h++) {
                double flight = 0;
                for (double[] spot : spots) {
                    if (fire(type, spot, RPM_MIN + i * RPM_STEP, h / (double) HOOD_STEPS)) {
                        r.count[i][h]++;
                        flight = Math.max(flight, lastFlight);
                    }
                }
                r.flight[i][h] = flight;
            }
        }
        return r;
    }

    // the middle of the longest run of good hoods at this rpm, -1 if there isn't one. {hood index, width}
    private static int[] bestHood(boolean[] good) {
        int bestStart = -1, bestLen = 0, start = -1;
        for (int h = 0; h <= good.length; h++) {
            boolean ok = h < good.length && good[h];
            if (ok && start < 0) start = h;
            if (!ok && start >= 0) {
                if (h - start > bestLen) {
                    bestLen = h - start;
                    bestStart = start;
                }
                start = -1;
            }
        }
        return bestLen == 0 ? new int[]{-1, 0} : new int[]{bestStart + (bestLen - 1) / 2, bestLen};
    }

    private void run() {
        int rpms = (RPM_MAX - RPM_MIN) / RPM_STEP + 1;
        long start = System.currentTimeMillis();
        Result[] pollen = new Result[DISTANCES.length];
        Result[] nectar = new Result[DISTANCES.length];
        for (int d = 0; d < DISTANCES.length; d++) {
            List<double[]> spots = spots(DISTANCES[d]);
            pollen[d] = sweep(SimCollisions.POLLEN, spots);
            nectar[d] = sweep(SimCollisions.RED_NECTAR, spots);
            // the most spots any rpm gets with both kinds of ball
            int needed = 0;
            for (int i = 0; i < rpms; i++) needed = Math.max(needed, Math.min(pollen[d].best(i), nectar[d].best(i)));
            pollen[d].markGood(needed);
            nectar[d].markGood(needed);
            StringBuilder angles = new StringBuilder();
            for (double[] spot : spots) angles.append(String.format(" %.0f", spot[3]));
            System.out.printf("%.0f in: %d of %d spots go in (angles:%s), %.0f s%n", DISTANCES[d], needed, spots.size(), angles,
                    (System.currentTimeMillis() - start) / 1000.0);
        }

        // how many distances each rpm can do with both balls. One rpm for everything is best, the flywheel
        // is the slowest thing on the robot
        int bestRpm = 0, bestScore = -1;
        for (int i = 0; i < rpms; i++) {
            int score = 0;
            StringBuilder row = new StringBuilder(String.format("%4d rpm:", RPM_MIN + i * RPM_STEP));
            for (int d = 0; d < DISTANCES.length; d++) {
                int p = bestHood(pollen[d].good[i])[1], n = bestHood(nectar[d].good[i])[1];
                row.append(String.format(" %d/%d", p, n));
                if (p > 0 && n > 0) score += 2 + Math.min(p, n);
            }
            System.out.println(row);
            if (score >= bestScore) { // ties go to more rpm: faster, flatter shots
                bestScore = score;
                bestRpm = i;
            }
        }

        System.out.println("---- paste into Shooter.SHOT_TABLE (distance, rpm, pollen hood, nectar hood, flight sec)");
        for (int d = 0; d < DISTANCES.length; d++) {
            int use = -1;
            for (int offset = 0; offset < rpms && use < 0; offset++) {
                for (int i : new int[]{bestRpm - offset, bestRpm + offset}) {
                    if (i < 0 || i >= rpms || use >= 0) continue;
                    if (bestHood(pollen[d].good[i])[1] > 0 && bestHood(nectar[d].good[i])[1] > 0) use = i;
                }
            }
            if (use < 0) {
                System.out.printf("// %.0f in: nothing gets both balls in from every spot%n", DISTANCES[d]);
                continue;
            }
            int p = bestHood(pollen[d].good[use])[0], n = bestHood(nectar[d].good[use])[0];
            System.out.printf(".add(%.0f, %d, %.2f, %.2f, %.2f)%n", DISTANCES[d], RPM_MIN + use * RPM_STEP,
                    p / (double) HOOD_STEPS, n / (double) HOOD_STEPS, pollen[d].flight[use][p]);
        }
        System.out.printf("took %.0f s%n", (System.currentTimeMillis() - start) / 1000.0);
    }

    public static void main(String[] args) {
        new ShotTableSweep().run();
    }
}
