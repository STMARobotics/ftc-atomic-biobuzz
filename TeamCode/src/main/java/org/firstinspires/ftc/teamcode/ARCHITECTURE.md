# How this code is organized

No commands and no scheduler. The robot is a few state machines run from one loop, in the style
of FRC 4414 HighTide (and 254/1678's "superstructure").

## The loop

Every OpMode runs the same four steps, in order, every loop:

```java
robot.readInputs();                                  // 1. read every sensor ONCE
robot.superstructure.setWantedState(WantedState.SHOOT); // 2. say what you want
robot.update();                                      // 3. superstructure decides
                                                     // 4. subsystems write to hardware
```

`Robot` is the "scheduler". You can read the whole robot's behavior top to bottom.

## Who does what

| File | Job |
|---|---|
| `Robot` | Owns all hardware, runs the four steps, turns on bulk reads |
| `Superstructure` | The brain. Turns a **WantedState** (what the driver/auto asks for) into a **SystemState** (what the robot is doing) and gives each subsystem a goal |
| `Subsystems/*` | Dumb. Each has a `Goal` enum, `readInputs()`, `writeOutputs()`. Reports facts like `atSpeed()`, never decides anything |
| `OpModes/ChunkMode` | TeleOp. Only maps gamepad buttons to wanted states |
| `OpModes/ShootAndLeaveAuto` | Auto as one enum state machine: entry action + exit condition per state |
| `Constants` | Every tunable number |

The drivetrain is driven directly by the OpMode/auto, not by the superstructure, so driving and
scoring never multiply into combo states.

## Command-based → state-based

| Used to be | Now |
|---|---|
| A Command | A value in an enum |
| CommandScheduler | The `while (opModeIsActive())` loop |
| Requirements / interrupts | One `wantedState` variable, so two things can't fight over a motor |
| Default command | The `IDLE` case |
| `whileActiveContinuous(cmd)` | `setWantedState(trigger ? SHOOT : IDLE)` every loop |
| `isFinished()` | A transition condition, e.g. `shooter.atSpeed()` |
| SequentialCommandGroup | Auto states that `enter()` the next one |

## How to add things

**A new mechanism (e.g. a turret):** make `TurretSubsystem implements Subsystem` with a `Goal`
enum, create it in `Robot` and add it to the `subsystems` list, then set its goal from
`Superstructure.applyGoals()`.

**A new robot behavior (e.g. eject):** add it to `WantedState`, add any new `SystemState`s, add a
rule in `nextState()`, add what each subsystem does in `applyGoals()`, and bind a button in `ChunkMode`.

**A new auto step:** add an enum value, an entry action in `enter()`, and an exit condition in
`runStateMachine()`.

## Rules

- **Never block.** No `sleep()` or `while` loops inside a state. Use an `ElapsedTime` and check it
  each loop.
- **Read hardware only in `readInputs()`.** Hardware reads are the slowest thing in FTC. Check
  "Loop time (ms)" on telemetry.
- **Subsystems don't talk to each other.** If the intake needs to know about the shooter, that
  logic goes in the superstructure.
