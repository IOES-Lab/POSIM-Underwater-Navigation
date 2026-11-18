# POSIM Underwater Navigation

Independent, measured-sensor underwater-navigation components for POSIM.

**Development status:** experimental library, not a qualified vehicle navigation
stack. No supported aircraft, surface vessel or underwater vehicle is certified
by these tests. Do not connect this candidate to a physical vehicle controller.

## Available components

- Raw-IMU preintegration and an error-state filter with DVL observations.
- Moving/stationary initialization primitives with explicit calibration priors.
- A bounded IMU-coast session for short DVL bottom-lock interruptions.
- Candidate horizontal GNSS/local-trajectory alignment without a fitted scale.

The library uses measured inputs and covariance. It does not read simulator truth
pose, infer an absolute origin from spawn position, or change raw GNSS fix status.

## Installation and tests

Python 3.10 or newer and NumPy are required. Neither ROS nor POSIM-SLAM is required.

```sh
python -m pip install .
python -m unittest discover -s tests
```

The tests cover numerical fixtures and invalid-input behavior. They do not prove
native sensor delivery, FCU acceptance, installed-app behavior or mission safety.

## Navigation behavior and limits

`BoundedInertialDvlSession` retains inertial prediction during a valid short
bottom-lock-loss report. It does not create a synthetic zero-velocity DVL sample.
Actual propagated position, velocity and attitude uncertainty are checked before
candidate output is returned. The development defaults are:

- Maximum coast horizon: 1.5 seconds since accepted DVL aiding.
- Position uncertainty: 5 m at three sigma.
- Velocity uncertainty: 0.5 m/s at three sigma.
- Attitude uncertainty: 5 degrees at three sigma.

These thresholds are conservative development parameters, not measured accuracy
claims. Missing IMU, malformed inputs, time reversal and exceeded confidence
limits invalidate the segment. Returning DVL must pass the measurement innovation
check. This library never sets vehicle `navigation_ready` to true.

`align_planar_tracks` accepts time-matched metric GNSS/local trajectory pairs.
The caller must handle GNSS projection, antenna lever arms, freshness and frame
identity. Eight samples and a 15 m baseline are default candidate requirements.
GNSS alignment is not an FCU source selection or an automatic mission restart.

## Integration boundary

The intended dependency direction is **POSIM-SLAM → Underwater-Navigation**.
Navigation is independent of SLAM. Existing POSIM-SLAM code is not changed by this
repository. Its initial numerical core is retained here with provenance in
[NOTICE](NOTICE); ownership migration is coordinated separately to avoid divergent
implementations.

The application owns sensor transport, session isolation, approved provider
registration, FCU source acknowledgments, mission decisions and presentation.
[Architecture and roadmap](docs/architecture.md) separates current code from
planned integration. [한국어 안내](README.ko.md).

## License

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
