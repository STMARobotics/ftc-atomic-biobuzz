package sim;

import org.firstinspires.ftc.teamcode.RobotConstants.Drive;
import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;
import org.firstinspires.ftc.teamcode.RobotConstants.Turret;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

/**
 * What the viewer's settings page can change while the sim runs. These are the robot code's real constants
 * (they just aren't final), so the running OpMode uses a new value on its next loop. It only changes this sim:
 * the page lists what's different so it can be copied into RobotConstants for the real robot.
 *
 * Only list something the code reads every loop. Things read once at INIT would look broken.
 */
final class SimSettings {

    private static final class Setting {
        final String group, label, help, name;
        final Field field;
        final boolean simOnly;
        final double min, max, step, defaultValue;

        Setting(String group, String label, String help, Class<?> owner, String fieldName,
                double min, double max, double step, boolean simOnly) {
            this.group = group;
            this.label = label;
            this.help = help;
            this.name = owner.getSimpleName() + "." + fieldName;
            this.min = min;
            this.max = max;
            this.step = step;
            this.simOnly = simOnly;
            try {
                field = owner.getDeclaredField(fieldName);
                field.setAccessible(true);
                defaultValue = read();
            } catch (ReflectiveOperationException e) {
                throw new IllegalStateException("No setting " + name, e);
            }
        }

        boolean isToggle() {
            return field.getType() == boolean.class;
        }

        double read() throws IllegalAccessException {
            return isToggle() ? (field.getBoolean(null) ? 1 : 0) : field.getDouble(null);
        }

        void write(double value) throws IllegalAccessException {
            if (isToggle()) field.setBoolean(null, value != 0);
            else field.setDouble(null, Math.max(min, Math.min(max, value)));
        }
    }

    private static final String SNAP = "Heading snap", DRIVE = "Driving", AIM = "Aiming", SEARCH = "Finding tags", FIELD = "Field";

    private static final List<Setting> ALL = new ArrayList<>();

    private static void toggle(String group, String label, String help, Class<?> owner, String field) {
        ALL.add(new Setting(group, label, help, owner, field, 0, 1, 1, false));
    }

    private static void slider(String group, String label, String help, Class<?> owner, String field,
                               double min, double max, double step) {
        ALL.add(new Setting(group, label, help, owner, field, min, max, step, false));
    }

    static {
        toggle(SNAP, "Snap to the nearest 90", "Only when driving, not intaking, shooting or turning", Drive.class, "HEADING_BIAS");
        slider(SNAP, "Strength", "Turn power per degree off", Drive.class, "BIAS_kP", 0.005, 0.15, 0.005);
        slider(SNAP, "Top turn power", "How hard it can turn. Past about 0.7 it stops helping", Drive.class, "BIAS_MAX_TURN", 0.1, 1, 0.05);
        slider(SNAP, "Stick needed", "How far over the left stick has to be", Drive.class, "BIAS_MIN_STICK", 0.1, 1, 0.05);
        slider(SNAP, "Wait after turning (s)", "After letting go of the turn stick", Drive.class, "BIAS_DELAY_SEC", 0, 1.5, 0.05);

        slider(DRIVE, "Stick curve", "1 is linear, 2 means half stick is quarter speed", Drive.class, "STICK_EXPONENT", 1, 3, 0.1);

        toggle(AIM, "Lead shots while driving", "Off just points at the target", Shooter.class, "LEAD_SHOTS");
        slider(AIM, "Most lead (deg)", "More and the tags leave the frame", Shooter.class, "MAX_LEAD_DEG", 0, 40, 1);
        slider(AIM, "Aim needed to shoot (deg)", "How close the turret has to be before the gate opens", Turret.class, "SHOOT_TOLERANCE_DEG", 0.5, 15, 0.5);
        slider(AIM, "Shoot anyway after (ms)", "RT with no tag in view", Shooter.class, "AIM_WAIT_MS", 0, 10000, 250);

        toggle(SEARCH, "Search when it loses the tag", "Sweeps the turret around", Turret.class, "SEARCH_FOR_TAGS");
        slider(SEARCH, "Sweep speed (deg/s)", "Too fast blurs the tags on a real camera", Turret.class, "SEARCH_DEG_PER_SEC", 30, 400, 10);
        slider(SEARCH, "Wait before searching (s)", "", Turret.class, "SEARCH_AFTER_SEC", 0, 3, 0.05);

        ALL.add(new Setting(FIELD, "Hive stiffness", "Sim only. How hard a Hive holds at its end. Under about 0.20 the staged nectar tips it",
                SimCollisions.class, "bistableTorque", 0.20, 0.50, 0.01, true));
    }

    private SimSettings() {}

    /**
     * Sets one by its name, like Drive.BIAS_kP. False if there's no such setting
     */
    static boolean set(String name, double value) {
        for (Setting s : ALL) {
            if (!s.name.equals(name)) continue;
            try {
                s.write(value);
                return true;
            } catch (IllegalAccessException e) {
                throw new IllegalStateException(e);
            }
        }
        return false;
    }

    static double get(String name) {
        for (Setting s : ALL) {
            if (!s.name.equals(name)) continue;
            try {
                return s.read();
            } catch (IllegalAccessException e) {
                throw new IllegalStateException(e);
            }
        }
        throw new IllegalArgumentException(name);
    }

    static String json() {
        Json j = new Json().beginArray();
        try {
            for (Setting s : ALL) {
                j.begin()
                        .field("name", s.name).field("group", s.group).field("label", s.label).field("help", s.help)
                        .field("toggle", s.isToggle()).field("simOnly", s.simOnly)
                        .field("min", s.min).field("max", s.max).field("step", s.step)
                        .field("default", s.defaultValue).field("value", s.read())
                        .end();
            }
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
        return j.endArray().toString();
    }
}
