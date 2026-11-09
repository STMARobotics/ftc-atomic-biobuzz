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

## The website

`sim\site.bat` puts it online at https://biobuzz.owenrossing.me while the window is open (Cloudflare
tunnel `biobuzz-sim` -> `SimGateway` on localhost:8090). Everything is behind an invite code, including
the CAD models. Codes are in `sim/site/invite_codes.txt` (not in git), one per line: `CODE name`.
Restart the site after editing them. Each person gets their own sim process, up to 6
(`SITE_MAX_PLAYERS`). A closed tab's field stops after 3 minutes, or 20 seconds if the site is full.

## Physics

Driving is per-wheel force: motor curve -> traction limit -> mecanum mix (`SimPhysics.stepDrive`). Contacts
use ode4j at 400 Hz (`SimCollisions`) with field shapes generated from the CAD
(`sim/cad/field_colliders.py`). Open http://localhost:8080/?colliders to see them over the model.
The stall, traction, drag and battery numbers are not checked against the real robot yet.

## Game pieces and Hives

The field starts as the manual's section 10.3.1 says, positions from the CAD. Balls are real spheres (`SimCollisions`)
with air drag. The deployed, running intake picks up what reaches its mouth, up to 4 (G407). Shots leave the barrel
the passive sorter would use, with speed from flywheel rpm and angle from that barrel's hood. Hives are hinged, hold
either end (bi-stable, per the manual) and tip when enough weight lands in the upward cell. The panel shows tips
and balls in cells (20 and 2 points, manual table 10-2).

Guesses to replace with real numbers: `BALL_EXIT_FRACTION` (film one shot), hood angle per servo position,
ball masses, `BISTABLE_TORQUE` (how many balls it takes to tip). `aShotCanScoreFromSomewhere` is a disabled
tuning test, re-enable it once the exit speed is measured: shots don't land in a cell yet.

## Not modeled (yet)

Nectar in the alliance areas is staged but can't come in yet (human player).
The shot table in RobotConstants comes from `gradlew -p sim shotTable`, which fires real shots through the physics and prints lines to paste in. Rerun it when the shot physics change, and replace it with ShotTuner numbers from the real robot.

The Settings button changes the robot code's real constants live in this sim (`SimSettings.java` lists them, add more there). Your browser remembers them. It never changes the real robot, so the page lists what's different to copy into RobotConstants.

Sounds are made in the browser from the state the sim sends (`web/sounds.js`), so there are no audio files. They're off until you turn them on with M or the button. The Musical checkbox picks between sounds in one key on a steady beat, and rougher ones closer to a real robot.

The Alliance menu picks whose tags the turret tracks, same as B (red) or X (blue) on the gamepad during INIT.

The Limelight sees tags but not what's in front of them. Ball spin, nectar unlocking on a tip, the sorter jamming.
The robot's outline is three boxes and the turret part doesn't rotate.
Pedro will follow paths better here than on the robot until its constants are tuned.
