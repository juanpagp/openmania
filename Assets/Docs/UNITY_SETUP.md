# Unity setup — install, verify, and integrate

## Install

1. Back up your project (or start a fresh one).
2. Delete (or move away) the old `Assets/` copy of this package.
3. Unzip this archive into the project so you have `Assets/Source`,
   `Assets/Scenes`, `Assets/StreamingAssets`, `Assets/Tests`, `Assets/Docs`.
4. Open the project in Unity 6000.x and let it import. The core compiles
   as `Tmnf.Sim` (no engine references); the scene drivers compile as
   `Tmnf.Sim.Unity`.

Requirements: nothing else. C# 9 is pinned via the asmdefs; there are no
packages, no unsafe code, no editor dependencies.

## Verify (two minutes)

Open each scene and press Play. On a healthy install you should see:

**`Scenes/TmnfSimSmoke.unity`**

```
[TmnfSimSmoke] SMOKE PASS — facade hash 0x5A45761F3728DF35 matches the golden data (400 ticks, bit-exact). ...
```

**`Scenes/TmnfSimPureData.unity`**

```
[TmnfSimPureData] facade check PASS — hash 0x5A45761F3728DF35 matches the golden trace (400 ticks, bit-exact, zero files). ...
[TmnfSimPureData] lap finished at tick 793 — finish time 7940 ms, checkpoints 1, laps 1. ...
```

**`Scenes/TmnfSimDiagnostics.unity`**

```
[TmnfSimDiagnostics] raw fp fingerprint 0x6568EC7748582DA6 (reference 0x382ABE95F7C1C311: MISMATCH — runtime promotes f32 trees to f64)
[TmnfSimDiagnostics] raw kernel 'association'[2] differs by one ulp: ... (informational)
[TmnfSimDiagnostics] guarded fp fingerprint 0x382ABE95F7C1C311 (reference 0x382ABE95F7C1C311: MATCH)
[TmnfSimDiagnostics] facade hash 0x5A45761F3728DF35 — the full 400-tick trace is bit-exact on THIS runtime (promotion-immune)
[TmnfSimDiagnostics] SUMMARY — raw fp: PROMOTING (informational, expected on Unity Mono), guarded fp: BIT-EXACT, facade trace: BIT-EXACT. Verdict: PASS — this runtime reproduces the golden trace.
```

The raw-battery MISMATCH is expected on Unity's Mono — it characterizes
the runtime, not the port. The verdict lines are the guarded battery and
the facade trace; both must be BIT-EXACT. If either ever is not, paste
the scene's whole console output into a report — it prints everything
needed to localize the first divergent tick and field.

The test suite runs in Unity via Window → General → Test Runner →
PlayMode (88 tests; on Unity's Mono the two raw-FP tests report a pass
message noting the promoting environment, everything else is straight
green).

## Integrate

Drive the simulation from your own components:

```csharp
// from files (StreamingAssets) — see TmnfSimSmoke.cs
Sim sim = Sim.Load(trackPath, vehiclePath, new SimOptions {
    fakeContactMask = maskPath, route = null, pinProvenance = false,
});

// or from pure data — see TmnfSimPureData.cs
TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoadBytes(bytes, null);
Sim sim = new Sim(track, vehicleBytes, maskBytes, new SimOptions { ... });
```

Step at the game's fixed rate (the sim is 100 Hz — one `Step` per
`FixedUpdate`, or use `TmnfSim.cs`'s frame-decoupled driver):

```csharp
TmnfInput input = TmnfInput.Digital(throttle, brake, steer, isRespawn);
VehicleState state = sim.Step(ref input);
// position, quaternion, velocities, wheels, engineRpm, gear — all f32
```

Convert between sim and Unity coordinates with `TmnfCoordinates`
(handedness conversion). Do not drive the sim from `Update()` with
variable dt — the tick contract is fixed.

## Determinism rules for your own code

If you extend the core and want to stay bit-exact, follow the same rules
the port follows (enforced in `ForbiddenApiScan` where possible):

- never use `MathF` / `UnityEngine.Mathf` inside the core,
- write constants from hex bit patterns, not decimal literals,
- wrap every float arithmetic op in `(float)(double)(...)` — the guard is
  identity on conforming runtimes and restores per-op rounding on
  Unity's promoting Mono (full story: `FLOAT_PROMOTION.md`),
- integer overflow is Rust-release semantics: wrap, don't check.

## Troubleshooting

| Symptom | Meaning |
| --- | --- |
| Smoke/PureData/Diagnostics all report the golden hash | everything is healthy |
| Raw battery mismatches by one ulp in `association` rows | expected on Unity Mono — informational only |
| Guarded battery or facade trace diverges | should be impossible with this core — report the scene output |
| "Missing (Mono Script)" on a scene object | the `Assets/` copy was mixed with a previous version — re-import from this archive's `Source/Unity/*.cs.meta` files |
