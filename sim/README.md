# BioBuzz sim

Runs the **real** TeamCode OpModes (ChunkMode, autos, tuners) on your PC against fake hardware,
with a 3D view you drive with a controller. No port: the same Java that goes on the robot.

## Run it

Double-click `sim\run.bat`, or from the repo root:

```
.\gradlew -p sim run
```

It opens http://localhost:8080. Pick an OpMode, press INIT, then START, like the Driver Station.
Plug in a controller and press any button so the browser sees it (Xbox and PS both work).
No controller: WASD drive, Q/E turn, Shift intake, Space shoot, F outtake, X battle mode,
R gate override, G flower, arrows for the dpad.

The first run downloads nothing new (it uses the same Gradle cache as the robot build), but the
page loads three.js from a CDN, so you need internet the first time you open it.

## Automatic tests (run before every comp)

```
.\gradlew -p sim test
```

Runs the real OpModes through scenarios in ~30 s and checks what physically happened: nothing
moves in TeleOp INIT, RT fires every ball and the shot counter matches, LT/battle mode, a weak
battery still shoots (low rpm fallback), a jammed ball gets backed out, unplugged devices are
reported instead of crashing, no flywheel encoder still shoots, auto drives its path and teleop
keeps auto's pose. Tests live in `src/test/java/sim/RobotBehaviorTest.java`. The sim can break
things on purpose for them: `world.unplug(name)`, `physics.batteryRestVolts`, `physics.jamNextBall`.

## What it checks for you

- **Moving during INIT**: any servo move or motor power before START shows a warning
  (marked as allowed in autos).
- **Crashes**: an exception in the OpMode shows the stack trace in the page.
- **Motor directions**: motors are "mounted" the way the real robot is, so flipping a
  `Direction` in the code makes the mechanism run backwards in the sim too. The two flywheel
  motors fight and the flywheel won't spin.
- **Hardware names**: the page lists every device the code asked for.

## How it works

- `sim/build.gradle` compiles `TeamCode/src/main/java` together with the sim, using the real
  FTC SDK, SolversLib and Pedro jars. The robot app build doesn't know the sim exists.
- `SimHardwareMap` hands out fake motors, servos, the OTOS, the turret encoder and the battery.
- `SimOpModeRunner` drives INIT/START/STOP the same way the Robot Controller app does.
- `SimPhysics` is a simple model of the robot: heavy drivetrain, the flywheel topping out around
  4000 rpm, the intake sagging the battery ~2V, servo travel time, and a toy ball model
  (3 preloads, one pickup per second while intaking) so shooting and the shot counter do
  something. Every number there is a guess, change it to match the real robot.

## Putting the CAD in

Every moving part is listed in `sim/web/robot.json` with its hinge and what drives it (a servo
position, a motor's rpm, or the turret angle). Right now they're placeholder boxes.

1. Export each moving part as its own `.glb`, all in the robot's coordinates (same origin:
   robot center on the floor, +x forward).
2. Put them in `sim/web/models/`.
3. Set `"file"` on the matching part in `robot.json`, plus `"scale"` (39.37 if it's in meters)
   and `"rotation"` if the CAD is Z-up (usually `[-90, 0, 0]`).
4. Refresh the page. No rebuild needed.

## Not modeled (yet)

The field elements (seesaw, goal), the Limelight, real ball physics, collisions with anything
but the walls. Pedro will follow paths better here than on the robot until its constants are tuned.
