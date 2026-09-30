# Benchmark Skeleton (draft)

A first draft of the API and task base class for the lab's robotics benchmark, built on Drake with CENIC. Written for group discussion, not final.

## Quick summary

The benchmark is split into three pieces: the Robot, the Task, and the Policy being tested. The main decisions:

1. The policy only sees what real sensors would see.
2. The policy lives outside the simulation.
3. Plain Python now, gRPC before outside users submit policies.
4. Always record how long the policy takes to answer.
5. The command format copies the real robot's.
6. A checker decides pass or fail after every step.
7. Seeds make every run repeatable.

Each one is explained below with its pros and cons.

## The big idea

Every decision below is about encapsulation: where we draw the walls between pieces, and what is allowed to cross them.

There are three pieces:

- **Robot** (benchmark side): one real robot, like the G1, the Trossen arms, or the LEAP hand. It defines what the robot can sense and what commands it accepts. It is written once and reused by every task for that robot.
- **Task** (benchmark side): the rest of the world, the random starting positions, and the pass/fail checker.
- **Policy** (user side): the controller or AI policy being tested. It only ever sees sensor readings and only ever sends commands.

They talk in a loop:

```
Benchmark: "Here's what the robot senses right now."
Policy:    "OK, do this."
Benchmark: runs physics a little, repeats, then judges pass or fail
```

## The decisions

### 1. The policy only sees what real sensors would see

The policy gets joint readings, IMU readings, and camera images. It never gets the simulator's hidden information, like exact object positions.

- **Pros:** Results reflect what could work on a real robot, and every policy is compared fairly.
- **Cons:** Each robot's sensor list has to be chosen carefully. For example, Drake knows the G1's pelvis position in the room, but the real G1 does not, so that must be left out. Scores will also be lower than in benchmarks that allow peeking.

### 2. The policy lives outside the simulation

The benchmark calls the policy on a timer and holds its command steady between calls. The policy never touches Drake.

- **Pros:** Users don't need to learn Drake, they can't peek inside, and a hand-written controller and a large AI policy plug in exactly the same way.
- **Cons:** Physics pauses while the policy thinks (see decision 4).

### 3. Plain Python now, gRPC before outside users submit policies

For now, the benchmark calls the policy directly. Later, the policy runs as its own program and talks to the benchmark over gRPC, like TRI's LBM-eval.

- **Pros:** Plain Python is fast to build and debug. Both versions use the same two calls, `reset` and `step`, so switching is a swap, not a redesign. gRPC later gives separate software setups (AI models often need software that clashes with Drake), separate machines, crash isolation, and makes cheating impossible.
- **Cons:** In plain Python, a determined user could get around the rules. gRPC adds setup work and a little communication time.

### 4. Always record how long the policy took to answer

The runner times every call to the policy and reports the average and the worst case.

- **Pros:** It's cheap to measure, and it shows when a policy (usually a large AI model) would be too slow on real hardware. It also allows an optional "realistic" mode later, where the policy's command arrives late.
- **Cons:** Timing depends on the computer, so it can't be compared directly between labs. A realistic mode would make runs non-repeatable, so it should stay optional.

### 5. The command format copies the real robot's

For the G1, each motor gets 5 numbers, just like the real hardware: target angle, target speed, stiffness (kp), damping (kd), and extra torque. Limits match the real motors.

- **Pros:** The benchmark makes no hidden choices. Pure torque control (stiffness and damping set to 0) and angle control are both covered. Policies speak the same language as the real robot, which makes hardware testing easier.
- **Cons:** The command is bigger (5 × 23 = 115 numbers per step for the G1), and each robot needs its own format. If limits don't match the real motors, results get inflated.

### 6. A checker decides pass or fail after every step

After each step, the task's checker says success, failure, or keep going. Running out of time counts as failure. The checker may see everything in the simulation, because it's the referee, not the player. Extra details like "how close it got" are recorded but don't change the score.

- **Pros:** The score is simple (a success rate over many starts), runs stop early once the robot has clearly failed, and the extra details help explain failures.
- **Cons:** Success conditions must be precise or results are unfair, and some are hard to write (what exactly counts as a landed backflip?). Each task's success and failure demos are needed to catch a broken checker.

### 7. Seeds make every run repeatable

Each run's random starting position comes from a seed number. Same seed, same start, same result.

- **Pros:** Everyone is tested on the same starts, and any failure can be replayed exactly.
- **Cons:** All randomness in a task must come from the seed, or repeatability quietly breaks. If the seeds are public, people can tune their policy to them. Seeds only change positions, not what objects are in the scene.

## What's in this repo

- `skeleton.py`: a rough draft of the three pieces above (`Robot`, `Task`, `Policy`) as Python classes, plus the loop that runs one test. It's meant to show the shape of the design, not a finished implementation.
