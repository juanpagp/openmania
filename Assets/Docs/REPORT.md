# Tmnf.Sim — the bit-exact C# port of the TMNF physics core

This package contains a complete port of the tmnf-physics simulation core
to plain C#, arranged as a drop-in Unity 6000.x `Assets/` folder. The port
is bit-exact against the Rust original: every golden fixture, every trace
file, and the 400-tick facade telemetry hash match byte-for-byte on every
conforming runtime, and — as of this build — on Unity's embedded Mono as
well (see `FLOAT_PROMOTION.md` for how that was achieved).

## What is in here

```
Assets/
  Source/Core/          the simulation core — zero Unity references
    Collision/          narrowphase + the game's collision record buffer
    Data/               byte-contract readers/writers (TGN1 golden format)
    Diagnostics/        the FP kernel battery + pinned reference table
    Dyna/               rigid-body dynamics (the 0x38-byte state blocks)
    Fixtures/           pure-data track/vehicle/route/mask construction
    Fp/                 the FP policy: bit utils, Fp helpers, the libm lane
    Math/               Vec3 + GmFunc (the game's scalar function library)
    Race/               checkpoint/race layer
    Response/           engine response model
    Route/              route (checkpoint graph) loader
    Sim/                the facade: Sim.Load / Sim.Step / VehicleState
    Surface/            surface materials
    Track/              TMNFTRK1 track loader + SHA-256 track hashing
    Vehicle/            the Stadium vehicle model (Model6 and friends)
    World/              world construction, ticks, response mediation
  Source/Unity/         the Unity layer: scenes' drivers, coordinates, driver
  StreamingAssets/
    Tmnf.Sim.GoldenData/  the golden fixtures and traces (checked in)
  Tests/Tmnf.Sim.Tests/  the 88-test NUnit golden suite (net8.0 + Unity
                         Play Mode via the same sources)
  Scenes/               TmnfSimSmoke / TmnfSimPureData / TmnfSimDiagnostics
  Docs/                 this report, UNITY_SETUP.md, FLOAT_PROMOTION.md
```

The core builds standalone with `dotnet build` (`netstandard2.1` +
`net8.0`, C# 9 — the level Unity 6000.x compiles at) and inside Unity via
`Tmnf.Sim.asmdef` (`noEngineReferences: true`).

## The byte contract

Determinism is defined by golden data, generated from the Rust original:

- **Fixtures** — `plain.tmnftrack`, `stadium.tmnfvehicle`, the route, the
  128×128 fake-contact mask. The pure-data builder (`TmnfFixtures`)
  reproduces all of them byte-for-byte from code, so the pipeline can run
  with zero files.
- **Traces** — TGN1 files: per-row input bytes in, output bytes out, over
  thousands of rows (`fp/`, `collision/`, `dyna/`, `gm/`, `race/`,
  `response/`, `route/`, `vehicle/`).
- **The facade trace** — `sim/facade.tgn`: the 400-tick drive schedule
  (accelerate → brake → accelerate + steer left) with a 104-byte telemetry
  record per tick:

```
0-3    raceTimeMs (u32)
4-15   position (3 × f32)
16-31  quaternion x,y,z,w (4 × f32)
32-43  linear velocity (3 × f32)
44-55  angular velocity (3 × f32)
56-95  wheels ×4 (u8 onGround, i32 material, u8 sliding, f32 spin)
96-99  engineRpm (f32)
100-103 gear (i32)
```

The FNV-1a 64 hash of that stream is the smoke contract:
`0x5A45761F3728DF35`.

## The determinism policy

- All core math goes through explicit f32/f64 policy helpers (`Fp`,
  `Libm`, `GmFunc`). `MathF` and Unity's `Mathf` are never used.
- Integer arithmetic wraps like Rust release builds (`Fp.RustAs*`,
  `CheckForOverflowUnderflow=false`).
- Constants are materialized from exact hex bit patterns
  (`BitConverter.Int32BitsToSingle(0x…)`), never from decimal literals.
- Every float arithmetic op is materialized through a `(float)(double)(…)`
  guard so every runtime rounds per-op — this is what makes the core
  immune to Unity Mono's f32→f64 expression-tree promotion. The why and
  the proof: `FLOAT_PROMOTION.md`.
- The FP kernel battery (`FpDiagnostics`) pins the runtime's primitive
  behavior against a generated reference table; the diagnostics scene
  reports it as environment characterization (raw) and verdict (guarded).

## The scenes

| Scene | What it proves |
| --- | --- |
| `TmnfSimSmoke` | end-to-end bit-exactness inside Unity: file-based fixtures, the full pipeline, the 400-tick facade hash. |
| `TmnfSimPureData` | the same contract with zero files (pure-data fixtures), plus a full race lap that finishes at tick 793 (7940 ms). |
| `TmnfSimDiagnostics` | environment verification: raw/guarded FP batteries + facade trace, with per-tick diff tooling should anything ever diverge. |

One upstream quirk to know about: the facade's `State()` observes
`canRespawn`, which panics on finished races. That panic marks the
finishing tick — read `sim.Race()` instead of stepping further. The
PureData scene demonstrates the pattern.

## The test suite

88 NUnit tests over the same golden data (run with
`dotnet test`, or in Unity via Test Runner → PlayMode):

- `FpGoldenTests` — the fp policy primitives (ftol, x87 sin/cos/exp/atan2/
  fmod, u32→x87 float) over 100k+ rows each.
- `CollisionGoldenTests`, `DynaGoldenTests`, `GmGoldenTests`,
  `VehicleGoldenTests` — per-module golden traces.
- `SimTraceTests` — whole-pipeline traces, including the facade hash.
- `FixturesTests` — pure-data fixtures byte-identical to the files.
- `ForbiddenApiScan` — enforces the determinism policy (no MathF, no
  Unity Mathf in the core, etc.).
- `FpDiagnosticsTests` — pins the runtime's FP primitives: raw battery
  (conforming runtimes; promoting runtimes pass via the guarded battery)
  and the guarded battery (must be bit-exact everywhere).

## Where to start reading code

1. `Sim/Sim.cs` — the facade you drive: `Sim.Load(...)`, `Step(...)`,
   `VehicleState`.
2. `Fixtures/TmnfFixtures.cs` — the whole world built from constants.
3. `Dyna/Dyna.cs` — the state blocks and integration.
4. `Fp/Fp.cs` — the arithmetic policy everything trusts.
5. `Docs/FLOAT_PROMOTION.md` — why one ulp in Unity's Mono mattered and
   how the core became immune.
