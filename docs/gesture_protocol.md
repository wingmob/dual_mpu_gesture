# Gesture Protocol

## Mounting

- Sensor 1: back of the hand
- Sensor 2: forearm, near the wrist
- Keep both sensor axes aligned in the same direction
- Fix them tightly; loose mounting will destroy repeatability

## Recording Rules

- Sit or stand in a stable posture
- Elbow angle should remain roughly constant
- Perform one gesture per trial
- Start from a neutral pose
- Return to neutral pose after the gesture
- Keep each gesture within about `1.0` to `1.5` seconds

## Suggested Label Set

- `flexion`
- `extension`
- `radial_deviation`
- `ulnar_deviation`
- `pronation`
- `supination`

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
