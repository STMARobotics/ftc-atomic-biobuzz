package sim;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * All the fake devices plus the physics. Devices live as long as the sim does, so the robot
 * stays where it is between OpModes like a real one.
 */
public final class SimWorld {

    public enum Phase { IDLE, INIT, RUNNING, STOPPED }

    volatile Phase phase = Phase.IDLE;
    volatile boolean autonomous = false;

    final SimPhysics physics = new SimPhysics(this);

    private final Map<String, SimMotor> motors = new LinkedHashMap<>();
    private final Map<String, SimServo> servos = new LinkedHashMap<>();
    private final SimOtos otos = new SimOtos(this);
    private final VoltageSensor batterySensor;
    private final AnalogInput turretEncoder;

    private final Set<String> requested = Collections.synchronizedSet(new LinkedHashSet<>());
    // fault injection: names in here act like they're missing from the Driver Hub config
    private final Set<String> unplugged = Collections.synchronizedSet(new LinkedHashSet<>());
    private final Set<String> warnings = Collections.synchronizedSet(new LinkedHashSet<>());

    public SimWorld() {
        batterySensor = Fakes.proxy(VoltageSensor.class, "battery", (method, a) ->
                method.equals("getVoltage") ? physics.batteryVolts : Fakes.NOT_HANDLED);
        AnalogInputController analog = Fakes.proxy(AnalogInputController.class, "analog", (method, a) -> {
            if (method.equals("getAnalogInputVoltage")) return physics.turretEncoderVolts();
            if (method.equals("getMaxAnalogInputVoltage")) return 3.3;
            return Fakes.NOT_HANDLED;
        });
        turretEncoder = new AnalogInput(analog, 0);
    }

    /**
     * Fault injection: the next OpMode's hardwareMap.get(name) fails like it's missing from the config
     */
    public void unplug(String... names) {
        unplugged.addAll(java.util.Arrays.asList(names));
    }

    public void plugEverythingIn() {
        unplugged.clear();
    }

    public SimPhysics physics() {
        return physics;
    }

    public Phase phase() {
        return phase;
    }

    public SimServo servoState(String name) {
        return servo(name);
    }

    public SimMotor motorState(String name) {
        return motor(name);
    }

    VoltageSensor batterySensor() {
        return batterySensor;
    }

    SimOtos otos() {
        return otos;
    }

    /**
     * Hands out a fake device for hardwareMap.get(type, name)
     */
    Object device(Class<?> type, String name) {
        if (unplugged.contains(name)) {
            requested.add(type.getSimpleName() + " \"" + name + "\" (UNPLUGGED)");
            return null;
        }
        requested.add(type.getSimpleName() + " \"" + name + "\"");
        if (DcMotorSimple.class.isAssignableFrom(type)) return motor(name).device;
        if (Servo.class.isAssignableFrom(type)) return servo(name).device;
        if (SparkFunOTOS.class.isAssignableFrom(type)) return otos;
        if (type == AnalogInput.class) return turretEncoder;
        if (VoltageSensor.class.isAssignableFrom(type)) return batterySensor;
        warn("The sim has no fake for " + type.getSimpleName() + " \"" + name + "\"");
        return null;
    }

    synchronized SimMotor motor(String name) {
        return motors.computeIfAbsent(name, n ->
                new SimMotor(n, SimPhysics.ticksPerRev(n), SimPhysics.mount(n), this));
    }

    synchronized SimServo servo(String name) {
        return servos.computeIfAbsent(name, n ->
                new SimServo(n, SimPhysics.servoRate(n), SimPhysics.servoStart(n), this));
    }

    synchronized List<SimMotor> motors() {
        return new ArrayList<>(motors.values());
    }

    synchronized List<SimServo> servos() {
        return new ArrayList<>(servos.values());
    }

    // ---- hooks from the fake devices ----

    void onMotorWrite(SimMotor motor, double power) {
        if (phase == Phase.INIT && power != 0) {
            warn(motor.name + " got power " + String.format("%.2f", power) + " during INIT" + initNote());
        }
    }

    void onServoWrite(SimServo servo) {
        if (phase == Phase.INIT) {
            warn(servo.name + " was moved during INIT" + initNote());
        }
    }

    private String initNote() {
        return autonomous ? " (allowed in auto)" : " (PENALTY in teleop)";
    }

    /**
     * The code set the localizer's pose during init. On a real robot that's where the drive team
     * put it, so the sim robot gets placed there too (unless it's the field origin, which is just
     * teleop zeroing its localizer).
     */
    void onOtosSetPosition(double x, double y, double heading) {
        if (phase != Phase.INIT) return;
        if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6) return;
        physics.placeRobot(x, y, heading);
    }

    /**
     * Fake hardware latency, only on the OpMode thread
     */
    void hardwareDelay(long ms) {
        if (!Thread.currentThread().getName().equals("OpModeThread")) return;
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    void warn(String message) {
        if (warnings.add(message)) System.out.println("[sim] WARNING: " + message);
    }

    public List<String> warnings() {
        synchronized (warnings) {
            return new ArrayList<>(warnings);
        }
    }

    List<String> requestedDevices() {
        synchronized (requested) {
            return new ArrayList<>(requested);
        }
    }

    /**
     * New OpMode: motors stop and reset, servos go limp, warnings clear. Positions stay.
     */
    synchronized void resetForNewOpMode(boolean isAutonomous) {
        autonomous = isAutonomous;
        for (SimMotor m : motors.values()) m.reset();
        for (SimServo s : servos.values()) s.reset();
        warnings.clear();
        requested.clear();
    }
}
