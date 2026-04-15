# Gesture Protocol

This project now supports two dual-`MPU6050` usage modes:

- `wrist mode`: back of the hand + forearm, for `flexion/extension/radial_deviation/ulnar_deviation/pronation/supination`
- `thumb/index mode`: thumb + index finger, for transition-based classification of `fist/open_palm/thumb_up/victory/point/ok/flexion/extension/radial_deviation/ulnar_deviation/pronation/supination`

## Shared Mounting Rules

- Keep both sensor axes aligned in the same direction
- Fix them tightly; loose mounting will destroy repeatability
- Add strain relief to the wires so cable pull does not move the boards
- After power-on or recalibration, keep both sensors still for a few seconds

## Wrist Mode

### Mounting

- Sensor 1: back of the hand
- Sensor 2: forearm, near the wrist

### Recording Rules

- Sit or stand in a stable posture
- Elbow angle should remain roughly constant
- Perform one gesture per trial
- Start from a neutral pose
- Return to neutral pose after the gesture
- Keep each gesture within about `1.0` to `1.5` seconds

### Suggested Label Set

- `flexion`
- `extension`
- `radial_deviation`
- `ulnar_deviation`
- `pronation`
- `supination`

## Thumb / Index Mode

### Mounting

- Sensor 1: thumb, dorsal side
- Sensor 2: index finger, dorsal side
- If you mount them on the nails, keep the boards low-profile and secure the wires carefully

### Recording Rules

- Wrist and forearm pose should stay roughly consistent across trials
- Perform one gesture transition per trial
- Start from a neutral hand pose
- Move into the target gesture
- Hold the target pose for about `0.5` to `1.0` seconds
- Relax back toward neutral before the trial ends
- Keep each gesture within about `1.5` to `2.0` seconds, with a bit of padding before and after

### Suggested Label Set

- `fist`
- `open_palm`
- `thumb_up`
- `victory`
- `point`
- `ok`
- `flexion`
- `extension`
- `radial_deviation`
- `ulnar_deviation`
- `pronation`
- `supination`

### Notes

- This mode is better at classifying `motion into a pose` than detecting a pose that has been static for a long time
- `victory` and `point` remain challenging because only thumb and index are instrumented
- Keep your neutral pose consistent, otherwise the absolute finger-pose features will drift
- For wrist labels in thumb/index mode, keep the fingers as still and repeatable as possible

## Minimum Dataset

- `20` trials per gesture for your own subject
- If possible, add `2` more subjects with `10` trials each
- Keep `10%` to `20%` of data as a held-out test split

## Win Conditions

- Real-time demo can continuously show current prediction
- Average response delay under `400 ms`
- Confusion matrix available
- Each gesture has a clear motion definition
- Hardware mounting photos and system diagram are prepared
