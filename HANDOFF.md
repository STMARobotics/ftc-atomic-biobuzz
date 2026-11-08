# Handoff: robot rewrite + 3D sim (2026-11-07)

Paste this file into a new Claude session (or say "read HANDOFF.md") to pick up where we left off.
Branch: `robot-rewrite`. `master` is untouched and is the fallback code for the qualifier.

## How to work with me (the programmer)

- Keep answers short and conversational, walls of text are hard to read.
- Don't commit or push without asking.
- I'm the only programmer and I'm vibe coding this robot: small changes, test each one, keep a
  known-good build to roll back to.
- Rule: **nothing may move during TeleOp INIT** (we got penalties). Servos and motors only move
  after START. Autos may stow the intake in their own init.

## The robot (FTC 2026-27 BioBuzz)

- Game pieces: pollen (2.8in wiffle ball) and nectar (3.6in wiffle ball), max 4 held. We shoot
  them into a seesaw target. The next comp is the qualifier (~early Dec 2026), and we can't work on
  the robot after it.
- Drive: 4x goBILDA 5203 312rpm at 1:1, standard 104mm goBILDA mecanum, 36 lb. OTOS is the only
  odometry. Pedro Pathing constants are still placeholders (needs the tuners run).
- Intake: surgical tubing, one 312rpm motor geared up to ~600rpm on TPU gears. Only the RIGHT
  motor is plugged in. Running it drops the battery ~2V. Pivot is an Axon MAX (TPU gears/mount, sloppy).
- Linear tubing indexer → servo gate → tubing slappers push balls through a passive sorter on the
  turret into a double-barrel shooter (one 3in flywheel for pollen, one for nectar). 2x 6000rpm
  5203 belted to the flywheels. It tops out ~4000 motor rpm and dips to ~3600 when the intake runs.
- Pollen and nectar hoods: underhand, 65-80 deg shot angle.
- Turret: 312rpm 5203 at 4:1, analog encoder through a gear stack, 0V = 0 deg, ~3.3V = ~540 deg.
  Not wired yet (Turret.ENABLED = false).
- Flower: a bar/ramp that deploys only as far as the intake (the intake is its hardstop) so pollen
  falls out of the flower tube into the intake. Disabled for now.
- Control Hub + Expansion Hub, Limelight 3A (Limelight.ENABLED = false until it's mounted), LEDs.
  No color sensors (on purpose).
- Seesaw read: the Limelight can't see the AprilTag when the seesaw is tipped toward us (can't
  score then). No HuskyLens.

## Controls (ChunkMode, gamepad 1)

Sticks drive (field centric), LT hold = intake, B hold = outtake, RT hold = shoot (spin up → gate
opens near speed → feeds), RB = gate override toggle, X = battle mode (cancel everything, fully
stow), A = flower toggle, dpad = hood nudges, START = reset field centric.

## What changed on robot-rewrite (compiles, NOT tested on the robot)

- Stayed with SolversLib commands (a mentor said a state machine isn't needed). Auto now uses the
  SolversLib scheduler too; Pedro's Ivy scheduler is gone from our code.
- `Robot.java` builds every subsystem once; `OpModes/RobotOpMode.java` is the base class (bulk
  reads on, loop timing). TeleOp and autos extend it.
- `Subsystems/IntakeSubsystem.java` replaces IntakePivot/IntakeWheels/Flower. Commands set
  requests (intake/feed/outtake/flower) and periodic() decides: outtake > feed > intake. Intake
  power ramps up over ~0.25s to soften the battery sag. The flower waits for the pivot to get down.
- Shooter: voltage-compensated feedforward (kS + kV*target)*(12/V) + kP*error. Fixed kP (was 1.0,
  now 0.0005). SHOOT_RPM is now 3700 (was 4500, which it could never reach). It counts shots from
  the rpm dip (for auto).
- Limelight: `seesawScorable()` = tag seen steadily for SEESAW_CONFIRM_MS.
- Drive stick curve is `Drive.STICK_EXPONENT` (2 = old squared feel, 1 = linear).
- Servos/motors only get written when the value changes.

**Must test on the robot:** retune shots in ShotTuner (the rpm changed, so the hoods need redoing),
check "Shots fired" goes up by exactly one per ball (tune SHOT_DIP_RPM), re-run JustMoveAuto.

## The 3D sim (`sim/`, see sim/README.md)

Runs the REAL TeamCode OpModes on a PC against fake hardware, with a Three.js view you drive with
a controller or keyboard. It has its own Gradle build, so the robot app build doesn't know it exists.

- Run: `sim\run.bat` (sets JAVA_HOME to Android Studio's JBR if it's installed in the default
  spot), or `.\gradlew -p sim run` with JAVA_HOME pointing at Java 17+. Opens http://localhost:8080.
  It listens on the LAN too (prints the address); `SIM_HOST=localhost` keeps it local.
- Verified on the PC: ChunkMode drives, intakes, shoots (the code's shot counter matched the sim),
  INIT-movement warnings work, JustMoveAuto gets placed at its start pose and follows its path.
- Key files: `SimOpModeRunner` (in the SDK's opmode package, drives INIT/START/STOP),
  `SimHardwareMap`, `SimWorld`, `SimPhysics` (all the guessed numbers), `SimOtos` (fake OTOS that
  Pedro's real localizer reads), `web/robot.json` (every moving part, its hinge and what drives it).
- The robot is placeholder boxes until the CAD is exported.

## Next up

1. **CAD into the sim.** Export each moving part as its own .glb (intake, gate, turret, both
   flywheels, both hoods, flower bar, plus the static chassis), all in the same robot coordinates
   (robot center on the floor, +x forward). Drop them in `sim/web/models/`, then work out each
   part's hinge/axis from the geometry and fill in `robot.json` (`file`, `scale`: 39.37 for meters,
   `rotation`: usually [-90,0,0] for Z-up CAD). Check by rendering and screenshotting the page.
   Also say in one line which way each mechanism moves so the directions aren't guessed.
2. **Field model** into the sim, lined up with Pedro coordinates (0-144 in).
3. **Seesaw auto**: intake, wait for `seesawScorable()`, shoot until 3 shots counted (or a timeout),
   and optionally go to the other side if the seesaw tipped the wrong way. Needs field poses.
4. Maybe: if the browser blocks the controller on a plain-http LAN address, serve the page over
   https or use a Chrome flag (keyboard still works).
