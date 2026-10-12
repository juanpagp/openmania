# OpenMania — a playable time-trial racing game on the TMNF physics port

This repository layers a complete, playable racing game on top of the
existing TrackMania Nations Forever vehicle-physics port (`Tmnf.Sim`).
The physics core is **untouched**: every bit of car movement still comes
from the ported simulation, driven through the same loaders the golden
traces verify. What has been added is everything around it — menus, a
data-driven track system, checkpoints, timing, personal bests, a results
flow, a placeholder car and debug collision rendering.

```
Assets/Source/Core      the untouched simulation (Tmnf.Sim assembly)
Assets/Source/Unity     the sim's Unity driver + handedness conversion (Tmnf.Sim.Unity)
Assets/Source/Game/Pure the engine-free bridge: Unity meshes -> simulation images
Assets/Source/Game/Runtime  gameplay: menus, race flow, track authoring components, HUD
Assets/Source/Game/Editor   track catalog tooling + authoring gizmos
Assets/Scenes           MainMenu, Race, and the two sample tracks (Assets/Scenes/Tracks)
Assets/Resources        TrackCatalog.asset (the track registration)
Assets/Tests/Tmnf.Game.Tests  the integration test suite (Unity Test Runner)
```

## 1. Opening and running

1. Open the project in **Unity 6000.6.5f1** (Unity 6.6; the editor version is
   pinned in `ProjectSettings/ProjectVersion.txt`).
2. Let it import (first import compiles three assemblies: `Tmnf.Sim`,
   `Tmnf.Sim.Unity`, `Tmnf.Game`).
3. Press **Play** in the `Assets/Scenes/MainMenu.unity` scene — the game
   entry point. (The scene is also first in the Build Settings; builds work
   the same way.)

The flow: **Main menu → Play → pick a track → countdown (3-2-1-GO) →
drive → finish → results → retry / track select / menu.**

The two shipped sample tracks (`Stadium Straight`, `Twin Finish`) are
regular track scenes built exactly the way you will build yours — they
double as templates and as examples for every feature (checkpoints, no
checkpoints, multiple finishes, ramps, walls).

### Verifying the physics port is intact

The original verification scenes still work unchanged:

- `Scenes/TmnfSimSmoke.unity` — golden facade hash from StreamingAssets files.
- `Scenes/TmnfSimPureData.unity` — the whole pipeline from pure data.
- `Scenes/TmnfSimDiagnostics.unity` — FP promotion diagnostics.

Expected output is documented in `Assets/Docs/UNITY_SETUP.md`.

### Running the tests

Window → General → Test Runner → **EditMode** tab → run all. The
`Tmnf.Game.Tests` suite drives real simulations through the whole bridge
(mesh conversion → loaders → collision → race logic → restarts). The same
test body also runs headless without Unity (see "Editor-less verification"
below), and passed 12/12 on the code shipped here.

## 2. Default controls

| Key | Action |
| --- | --- |
| W / Up | accelerate |
| S / Down | brake / reverse |
| A / D / Left / Right | steer |
| R / Backspace | restart the run from the start |
| Esc / P | pause menu (resume / restart / track select / main menu) |
| F3 | toggle collision debug rendering (the simulation's real colliders) |
| V | toggle the visible car model (debug rendering stays) |

The countdown freezes the simulation itself — the car physically cannot
fall, move or react to input until GO (the driver simply never steps the
sim; no input gating, no timeScale tricks).

## 3. Building a track

A track is an ordinary Unity scene. The recipe:

1. **Create the scene** anywhere under `Assets/` (the samples live in
   `Assets/Scenes/Tracks/`).
2. **Add the track root**: create an empty GameObject, add the
   **Track** component. Fill in:
   - **Track Name** — shown in the track list and results.
   - **Author**.
   - **Author Time (ms)** — the benchmark time; shown in the list and the
     HUD. Use `0` when there is none (the UI shows `--:--.---`).
   - *(optional)* **Apply Default Material On Load** — assigns a simple
     URP material to every renderer when the race loads; handy for
     greybox tracks, turn it off when your meshes bring their own
     materials.
3. **Add geometry**: create an empty GameObject under the root, add the
   **TrackGeometry** component, and parent your imported meshes under it.
   Every active MeshFilter in that hierarchy becomes collidable for the
   custom simulation — world position, rotation and scale are respected
   automatically. There is no per-mesh setup, no manual collider baking.
   - Alternatively (or additionally) you can tag objects with the
     **`Track`** tag; their subtrees are treated the same way.
   - **Read/Write Enabled** must be on for imported meshes (the converter
     reads their triangles when the race loads). The editor's import
     default already has it on; for player builds, keep it enabled for
     track meshes.
   - Surface friction: put the **TrackSurface** component on a mesh (or a
     hierarchy node) to pick one of the 31 TMNF physical materials —
     grass slows you down, ice barely grips, turbo pads accelerate. The
     hierarchy's TrackGeometry has a `Default Surface Material`
     (asphalt on the samples).
4. **Add gameplay objects** (see next section for details):
   - exactly one **StartBlock**,
   - zero or more **CheckpointBlock**s,
   - at least one **FinishBlock**.
5. **Register the track** in the catalog (see "Registering a track").
6. When you are happy with the run, set the **Author Time** metadata
   (your validation run's time is shown on the results screen).

A camera and light are not needed in track scenes — the Race scene
provides them (any camera in a loaded track scene is disabled
automatically). A light in the track scene is fine for editing, though:
it stays on while playing.

### Start / Checkpoint / Finish objects

All three are plain components you attach to any GameObject — build any
visual you like around them (custom mesh, hierarchy, particles); visuals
and gameplay are fully separate:

- **StartBlock** — exposes a *Spawn Point* (a Transform; auto-binds a
  child named `Spawn`). The car spawns there; the transform's **+Z
  (blue) axis is the direction the car drives toward**. Place it ~1 m
  above the road.
- **CheckpointBlock** — exposes one *Trigger Collider*: assign a
  **trigger-only BoxCollider** (the component's `Is Trigger` flag must be
  on — gameplay volumes never physically block the car). The car must
  pass through the volume; checkpoints can be collected **in any order**,
  and all of them must be collected before a finish counts.
- **FinishBlock** — same shape as a checkpoint. Triggering any finish
  completes the run once all checkpoints are collected. Several finishes
  are allowed (see `Twin Finish`'s two gates).

The editor draws gizmos for all three: a blue arrow for the start (the
driving direction), green boxes for checkpoints, red boxes for finishes,
cyan frames for TrackGeometry roots.

**Trigger tip** — the trigger volume detects the car while it *crosses a
face of the box* (within roughly a metre of any face, from outside). A
gate that spans the road (a few metres deep) is the natural shape; do not
expect a car spawned deep inside a huge box to register.

### Registering a track (track selection)

1. Select `Assets/Resources/TrackCatalog.asset` (create one via
   *Create > OpenMania > Track Catalog* if it is missing).
2. Add an entry, assign the **scene asset** (the runtime scene path is
   kept in sync automatically).
3. Refresh the cached display metadata with
   **OpenMania → Track Catalog → Refresh metadata from scenes** — it
   opens each scene in the background and copies name/author/author time
   from the Track component so the selection screen can show them
   without loading the scenes.
4. **OpenMania → Track Catalog → Sync scenes to Build Settings** puts the
   menu, race and every cataloged scene into the build's scene list
   (required for players; the Editor doesn't need it).

That's the whole workflow — no per-track code, no hardcoded lists: the
selection screen enumerates whatever the catalog holds.

## 4. Timing, personal bests and the author time

- The race clock is the simulation's own tick counter (100 Hz), started
  at the end of the countdown and stopped at a valid finish — it cannot
  be affected by menus, tab-outs or frame rate.
- **Personal bests** persist per track (keyed by scene path) in
  `persistentDataPath/openmania-pb.json` as a simple JSON table. A finish
  only overwrites the best when it is faster.
- The HUD shows the current time, the **live delta** against your PB
  (green when ahead, red when behind; falls back to the author time when
  no PB exists; hidden when neither exists), checkpoint progress and
  speed.
- The results screen shows the final time, your PB (with a *new best*
  marker), the author time and the delta against the reference.

## 5. The vehicle, its visuals and debug rendering

- The car is driven exclusively by the ported simulation. The visible car
  is a placeholder (box body, cabin, spoiler, four cylinder wheels) that
  follows the simulated state — including the **live wheel positions and
  spin** from the simulation's own wheel state.
- **Show Vehicle Visuals** (the `VehicleVisual` component's inspector
  flag, or the **V** key at runtime) hides the model without touching the
  simulation or the debug rendering.
- **F3** toggles the collision debug view: wireframe drawings of the
  collision shapes the simulation actually uses — the vehicle's body
  ellipsoids (cyan) and wheel ellipsoids (yellow) with their real radii
  and simulated transforms, the trigger volumes (green/red) and the start
  orientation (blue). It renders independently of the visible car, so you
  can hide the model and inspect the colliders alone.

## 6. How the Unity-to-simulation bridge works

For the curious (and for debugging your own tracks):

- `TrackSceneExtractor` walks the loaded track scene, validates the
  gameplay objects, reads every eligible MeshFilter and converts the
  triangles into TMNF-world-space mesh specs (the handedness conversion
  in `TmnfCoordinates` is the single place Unity space becomes sim
  space). Triangle winding is aligned with the mesh's authored normals,
  so **the solid side is the side you can see** — a floor is drivable
  from above, a flipped mesh is solid from below.
- `TmnfTrackImage` / `TmnfRouteImage` (pure C#, no engine references)
  serialize those specs into the simulation's on-disk image formats
  (TMNFTRK1 / TMNFROU1) **in memory** — including a BVH per mesh, a BVH
  over the entries, and the synthesized route centerline. The track then
  goes through the *real* loaders (`TmnfTrackLoadBytes` /
  `TmnfRouteLoadBytes`), so it gets the same validation, the same static
  grid acceleration and the same race logic the golden fixtures use.
- No dumped/generated files are ever required: the vehicle tuning and
  the fake-contact mask come from the repository fixtures
  (`Assets/StreamingAssets/Tmnf.Sim.GoldenData/fixtures/`), with the
  pure-data fixture builders as an automatic in-memory fallback.

**One important convention discovered while verifying the bridge**: the
reference game's ground entries carry *exact* (razor-thin) bounding
boxes, and the static broad phase relies on that — the car's tall front
body ellipsoid must not be tested against flat ground at normal ride
height (it only touches when the nose is genuinely pitched in — the TMNF
nose-scrape). The bridge therefore emits exact mesh AABBs and never
inflates them. If you author a floor mesh that is a razor-thin plane,
this is exactly the golden behaviour; if you author thick, chunky road
blocks, their boxes are as thick as the mesh.

## 7. Editor-less verification (what was tested without Unity)

The pure halves (simulation core + bridge + race logic) were compiled
and exercised headless with dotnet; the harness lives outside the
project but the checked-in test body is the same file the Unity Test
Runner uses (`Assets/Tests/Tmnf.Game.Tests/TrackPipelineChecks.cs`):

- the golden 400-tick facade hash still matches bit-exactly,
- bridged floors drive identically to the golden fixture (same positions
  over 300 ticks), wheels report ground contact,
- one-sided winding, baked non-uniform scale and a 400-mesh BVH grid all
  collide correctly,
- checkpoints collect in any order; a finish is rejected until every
  checkpoint is collected; zero-checkpoint and multi-finish tracks work,
- restarting from the cached images leaves no residual velocity, race or
  checkpoint state,
- time formatting and the personal-best JSON round-trip behave.

Everything else was validated statically: all generated scenes/assets
parse as Unity YAML, every script GUID and internal reference resolves,
MonoBehaviour serialization fields match their declarations, and the
catalog/build-settings scene lists point at existing files.

**Requires verification in the Unity Editor** (not possible in this
environment): scene import/Play-mode behaviour of the four scenes, uGUI
layout at various aspect ratios, the GL debug rendering under URP (F3),
the Input System keyboard paths, and the track-catalog editor menu
items. The code paths are straightforward and defensive; treat the first
editor open as a smoke test.

## 8. Known limitations

- **Last-checkpoint respawning is intentionally not implemented** — R
  restarts the whole run from the start (per the current design goal).
- Trigger volumes are **oriented boxes** (BoxCollider). Spheres/capsules
  as triggers are not converted.
- Track meshes must be readable at load time (Read/Write enabled) and
  static — the bridge snapshots them when the race loads; moving track
  geometry is not supported by the simulation's static collision model.
- The trigger detection has (inherited, game-accurate) semantics: it
  fires while the car is within about an ellipsoid radius of a face of
  the trigger box from outside. Extremely thin or extremely thick
  triggers still work, but spawning deep inside one will not register
  until you cross a face.
- Water is present in the track image but inert (level -1000). Surface
  materials are per-mesh, not per-face.
- The `Twin Finish` sample's second finish gate is rotated; rotated
  trigger boxes are supported (they are converted as oriented boxes).
- uGUI is built at runtime (no authored UI assets); if you restyle, do
  it in `UiBuilder`/`RaceHud`/`MainMenuController`.
