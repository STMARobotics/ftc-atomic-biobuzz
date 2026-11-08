package sim;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * The HardwareMap the OpMode gets. Every get() comes from the sim world instead of the hubs.
 * getAll() is empty, so there are no Lynx hubs and bulk reads are a no-op.
 */
public final class SimHardwareMap extends HardwareMap {

    private final SimWorld world;

    public SimHardwareMap(SimWorld world) {
        super(null, null);
        this.world = world;
        voltageSensor.put("Control Hub", world.batterySensor());
    }

    @Override
    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        T result = tryGet(classOrInterface, deviceName);
        if (result == null) {
            throw new IllegalArgumentException(String.format(
                    "Unable to find a hardware device with name \"%s\" and type %s (the sim doesn't fake this type)",
                    deviceName, classOrInterface.getSimpleName()));
        }
        return result;
    }

    @Override
    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        Object device = world.device(classOrInterface, deviceName.trim());
        return device == null ? null : classOrInterface.cast(device);
    }

    @Override
    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        return new ArrayList<>();
    }
}
