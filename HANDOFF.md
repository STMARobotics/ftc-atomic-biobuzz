# Handoff: learn-to-code tool plan + sim (2026-11-09)

Paste this file into a new Claude session (or say "read HANDOFF.md") to pick up where we left off. Don't write any code for this tool yet. I want to be part of the process.

## The project
FTC team BioBuzz, 2026-27 BIOBUZZ game. Branch: `robot-rewrite`. `master` is untouched. The repo holds the robot code (`TeamCode`, FTC SDK 12.0, SolversLib 0.3.6 commands, Pedro Pathing 3.0.1, Limelight 3A) and `sim/`, a 3D simulator that runs the real OpModes against fake hardware (ode4j physics, three.js page, controller input). `sim/README.md` explains how it works. Run it with `.\gradlew -p sim run`. Test it with `.\gradlew -p sim test` (43 tests, about 67 s).

Robot hardware and controls from Nov 7 are in `git show 9ebf023:HANDOFF.md`. Parts of that are out of date, so trust the code.

## The new idea
Turn the sim into less of a game and more of a tool (maybe an exe or Android Studio plugin) that lets newer students learn to code without a robot. I give a task like "make the intake spin when I press left trigger", they write the code, and the tool checks whether it worked.

The full plan is here. Read it with the docs tools, not a web fetch: https://claude.ai/artifact/JjaJWVRSVtkHnxiQKexCfm
I may have edited it or left comments there, and my edits win over this summary.

Recommended plan, none of it decided or built:
- Run lessons from the repo first and test them on one rookie. Then build a Windows exe with a built-in editor. Leave the plugin for last.
- Never run student code on my public website, because it would be running on my home PC.
- Each Run gets its own Java process: compile one student file, run it, and let a watchdog kill frozen code.
- I write tasks in a small task file. A checker presses buttons and reads what the robot did.
- 12 draft lessons, from telemetry up to commands on a real robot. Start with a simple starter robot.
- Phases: robot first until the qualifier, 1 prove it, 2 fill it out, 3 the exe, 4 plugin if wanted.

## What a throwaway test found
This was a temp-folder test on the desktop, not in the repo. It compiled sample OpModes and checked intake motor power, with no page or 3D view.
- Compile: about 450 ms cold, about 100 ms warm, with Java 8 rules (`--release 8`).
- Result: about 0.7 to 1.3 s from a fresh Java process.
- Correct code passed as both OpMode and LinearOpMode. A missing `else` failed correctly.
- A misspelled device name quietly creates a new motor. A real robot would crash, so this must change.
- A `while` loop inside `loop()` freezes the OpMode and the sim doesn't say so.
- `var` and `setpower` only give "cannot find symbol", so errors need plain English.
- A motor named `intake` doesn't turn the model's intake, because `SimPhysics` reads `intakeMotorRight`. The sim engine has 56 references to the robot code, 40 of them in `SimPhysics`.

Other blockers:
- Android's SDK terms (clause 3.4) don't allow shipping `android.jar` in the exe.
- The page loads three.js from a CDN, which won't work offline.
- The exe must not include `TeamCode`, my CAD or invite codes.
- There are no fake IMU, touch, color or distance sensors yet.
- Android Studio's Java has no `jmods`, so packaging needs a full JDK.

## Questions I haven't answered yet
1. What computers do students have: Windows, Mac or Chromebook? This changes the plan most.
2. Is it for my team's rookies first, or any FTC team?
3. Should I try Telemark (https://sharpfacerobotics.github.io/telemark) for 30 minutes first, in case it's enough?
The plan's "Decisions for you" table has the rest, with recommended defaults.

## State I left things in
- All the robot and sim work is pushed to `robot-rewrite`, including the robot CAD meshes (`sim/web/models/robot/`). This repo is public, and I chose to publish them.
- I'll probably move the sim out of this repo into a separate personal repo later, since it may become a plugin or similar. Don't move anything until I say so. Git history here will still have the old files after a move.
- The last full test run passed all 43 tests. Only web page changes (sounds, default camera, alliance) came after it.
- The website (biobuzz.owenrossing.me) runs from the desktop and is down. The gateway exited with code 4, and the log's last entry is 7:37 PM on Nov 8 with no error. It hasn't been restarted. `sim\site.bat` starts it, and that only works on the desktop.
- `sim/site/` and `sim/matchlogs` are git-ignored and exist only on the desktop.

## How I want you to work
- Don't commit or push without asking.
- Keep answers short and conversational. Walls of text are hard.
- Run tests in the background so we can keep talking.
- Don't move the repo or split it yet. That's only a conversation so far.
- Invite codes never go in chat or git. Don't read or print `sim/site/`.
- The robot comes first. The qualifier is around early December 2026, and we can't work on the robot after it.
- Nothing may move during TeleOp INIT. Servos move only after START.
- Stay with SolversLib commands, no state machines.
- I'm a solo programmer who vibe codes, so explain things plainly.

## First steps
1. Read the plan doc, including any comments.
2. Run `git status` and `git log -3` so you know what this machine has.
3. Ask me the unanswered questions, one or two at a time. Don't write code until I say so.
