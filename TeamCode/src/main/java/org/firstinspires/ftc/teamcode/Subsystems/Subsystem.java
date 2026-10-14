package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Every subsystem is "dumb": something else gives it a goal, and it reports facts back.
 * It never decides on its own when to intake or shoot. That's the superstructure's job.
 *
 * {@link org.firstinspires.ftc.teamcode.Robot} calls these once per loop, in this order:
 * readInputs() on everything, then the superstructure decides, then writeOutputs() on everything.
 */
public interface Subsystem {

    /** Read sensors into fields. Everything else in the loop reads those fields, not the hardware. */
    void readInputs();

    /** Push the current goal to the hardware. */
    void writeOutputs();

    void telemetrize(Telemetry telemetry);
}
