# Float promotion in Unity's Mono — why the v2 smoke scene diverged, and how the core is now immune

## TL;DR

- **v2 failed on one specific environment**: Unity 6000.6.5f1's embedded Mono.
  The 400-tick facade hash came out `0x1A1518C51AE7077B` instead of the
  golden `0x5A45761F3728DF35`, first diverging at tick 3 in the wheel spin
  (`3.00000024` vs golden `3.00000048` — exactly one ulp).
- **Root cause**: Unity's embedded Mono evaluates 32-bit float expression
  trees with 64-bit intermediates, rounding to f32 only at expression
  boundaries. Every runtime that rounds per-operation — desktop .NET,
  upstream Mono's CLI with default flags, even Mono's interpreter —
  produces the golden trace. The C# port was never wrong; the runtime
  computes f32 trees differently.
- **The fix (this build)**: every float arithmetic operation in the core is
  explicitly materialized through a `(float)(double)(...)` guard — 1,689
  guards across 17 files, applied by a deterministic Roslyn transform.
  This is provably identity on conforming runtimes and provably
  restorative on promoting runtimes. The facade trace is now bit-exact on
  every runtime we can construct, including an exact bit-for-bit proxy of
  the failing Unity editor.

## The evidence trail

The `association` kernel in `FpDiagnostics` (a = 0.3333333432674408f,
b = 87.9932556f, c = 32.4543343f) isolates the mechanism in one line:

| expression | per-op f32 (golden) | f64-promoted (Unity) |
| --- | --- | --- |
| `(a * b) * c` | `0x446DFAEF` | `0x446DFAF0` (+1 ulp) |
| `a * (b * c)` | `0x446DFAEF` | `0x446DFAF0` (+1 ulp) |
| `(a + b) + c` | `0x42F18FD6` | `0x42F18FD5` (−1 ulp) |

A runtime that computes each operation in f64 and rounds once at the end
of the expression tree gets the promoted values; a runtime that rounds
after every operation gets the golden values. One ulp is invisible in
almost any application — but the facade trace is a zero-tolerance byte
contract, and the drift amplifies through 400 ticks of dynamics until
positions visibly disagree.

## The mechanism

Upstream Mono's JIT has an optimization literally called `float32`: keep
f32 values in 32-bit SSE registers and round after every f32 operation.
It is enabled by default when you run the `mono` CLI. Unity embeds Mono
as a library and never enabled that optimization, so Unity's Mono:

1. widens every f32 to f64 in registers,
2. computes entire f32 expression trees in double,
3. rounds back to f32 only at stores, call boundaries, and other
   expression boundaries.

This is a known, publicly documented behavior. Miguel de Icaza (Mono's
creator): *"In Mono, decades ago, we made the mistake of performing all
32-bit float computations as 64-bit floats while still storing the data
in 32-bit locations. Official Mono releases have switched… Unity never
enabled that functionality."* Aras Pranckevičius (Unity), writing about
float-precision mysteries in the editor: *"The Mono C# runtime used in
Unity does all math in double precision, everywhere."*

Two consequences matter for a bit-exact port:

- **A single operation is still IEEE-correct.** Computing one `+`, `-`,
  `*`, `/`, or `%` on two f32 values in f64 and rounding once to f32
  yields the correctly-rounded f32 result — the f64 significand (53
  bits) has at least `2·24 + 2` bits, so the intermediate f64 rounding
  can never create a double-rounding error for a single f32 operation
  (Figueroa's classic result). `%` is exact for widened f32 operands.
  This is why every golden suite outside Unity matched: most library
  code (including this port's double-precision `Libm` lane) only ever
  does single f32 ops between f32 materializations.
- **Expression trees lose their intermediate roundings.** `(a * b) * c`
  computed as `double(a)·double(b)·double(c)` rounded once is *not*
  `float(float(a·b)·c)`. That is the entire divergence.

## The fix: per-op materialization

Every float-typed arithmetic binop in the core is now wrapped:

```csharp
// before
x = (a * b) + c;

// after — the v3 core, everywhere
x = (float)(double)((float)(double)(a * b) + c);
```

Why this works:

- On a **promoting runtime** the cast pair emits a real `cvtsd2ss` per
  operation — the value is forced back into an f32 register materialization
  before the next operation consumes it. Every intermediate is rounded
  exactly once, per operation, which is precisely the conforming
  semantics.
- On a **conforming runtime** the guard is identity: the operand was
  already an f32; widening it to f64 and narrowing it back returns the
  same bits (the operations are exact for already-rounded values), and
  single-op f64 computation with one final rounding is the correctly
  rounded f32 result by the same Figueroa argument. The identity is not
  just theoretical — all 88 golden tests pass bit-for-bit on .NET 8 and
  on stock Mono with default flags after the transform.
- The guard must be applied **uniformly, to every f32 binop, no
  exceptions**: on a promoting runtime, f32 *locals* are not rounding
  sinks (the unrounded f64 value can live in a register across
  statements), so there is no smaller set of "safe" places to guard.
  Every materialized value therefore provably holds a rounded f32.

The transform is mechanical and deterministic: a Roslyn-based rewriter
selects every binary expression of type `float` with an arithmetic
operator (`+ - * / %`) and inserts the guard text — 1,689 wraps across
17 core files. It also fixes a subtler trap: `(double)(a * b)` now widens
the *rounded* product, matching what conforming runtimes produce for the
same expression.

`FpDiagnostics.cs` intentionally keeps its RAW battery unguarded — that
battery is the environment detector. Its sibling
`FpDiagnostics.RunKernelsGuarded()` runs the same kernels with the same
guards as the core and must match the same reference table on every
runtime.

## The proof

Stock Mono with the `float32` optimization switched off
(`mono -O=-float32`) is a bit-exact proxy for Unity's embedded Mono. The
four-way experiment (raw vs guarded core, default vs promoting JIT):

| core | JIT mode | facade hash | tick-3 wheel spin |
| --- | --- | --- | --- |
| raw | default (per-op f32) | `0x5A45761F3728DF35` — golden | `0x40400002` (golden) |
| raw | `-O=-float32` (Unity-like) | `0x1A1518C51AE7077B` — the failure | `0x40400001` (−1 ulp) |
| guarded | default | `0x5A45761F3728DF35` — golden | `0x40400002` (golden) |
| **guarded** | **`-O=-float32`** | **`0x5A45761F3728DF35` — golden** | **`0x40400002` (golden)** |

The promoting column of the raw row reproduces the failing Unity editor
**bit-for-bit**: same raw FP fingerprint (`0x6568EC7748582DA6`), same
facade hash, same first-divergence field. The guarded row is golden
everywhere — including under the promoting JIT.

Beyond the proxy, the full 88-test golden suite (fp, collision, dyna,
vehicle, route, sim traces) passes on .NET 8 after the transform, and the
guarded FP battery matches the conforming reference table on every
runtime tested: .NET 8, stock Mono default flags, stock Mono with
`-O=-float32`, and the Mono interpreter.

## What the diagnostics scene reports now

`Assets/Scenes/TmnfSimDiagnostics.unity` reports three things:

1. **Environment** — Unity version, runtime, CPU.
2. **RAW FP battery** — environment characterization. On Unity's Mono
   this mismatches the reference by one ulp in the `association` rows and
   prints `PROMOTING`. That is expected and *informational*.
3. **GUARDED FP battery + facade trace** — the verdicts. Both must be
   bit-exact. With this core they are, on every runtime we can construct.

Expected console output on a healthy Unity 6000.x install:

```
[TmnfSimDiagnostics] raw fp fingerprint 0x6568EC7748582DA6 (reference 0x382ABE95F7C1C311: MISMATCH — runtime promotes f32 trees to f64)
[TmnfSimDiagnostics] raw kernel 'association'[2] differs by one ulp: got 0x446DFAF0, reference 0x446DFAEF — f32 tree promotion (informational)
... (rows [3] and [4] likewise) ...
[TmnfSimDiagnostics] guarded fp fingerprint 0x382ABE95F7C1C311 (reference 0x382ABE95F7C1C311: MATCH)
[TmnfSimDiagnostics] all 10 guarded fp kernels bit-exact vs the conforming-runtime reference — per-op f32 rounding is active on this runtime
[TmnfSimDiagnostics] facade hash 0x5A45761F3728DF35 — the full 400-tick trace is bit-exact on THIS runtime (promotion-immune)
[TmnfSimDiagnostics] SUMMARY — raw fp: PROMOTING (informational, expected on Unity Mono), guarded fp: BIT-EXACT, facade trace: BIT-EXACT. Verdict: PASS — this runtime reproduces the golden trace.
```

(On a conforming runtime the raw battery matches too, and the summary
reads `raw fp: conforming`.)

## Performance

On promoting runtimes each guard costs one extra `cvtss2sd`/`cvtsd2ss`
pair per f32 operation. The 400-tick facade drive executes in
milliseconds; the full golden suite shows no measurable slowdown. On
conforming runtimes the JIT folds the identity cast pair away.

## Reproducing the proxy outside Unity

```sh
mono  TmnfMonoCheck.exe            # per-op f32   -> golden hashes
mono -O=-float32 TmnfMonoCheck.exe # Unity-like   -> the failure hashes on a raw core, golden on this core
```

Any stock Mono 6.x exhibits the split; the flag exposes the same JIT
machinery Unity leaves off. This is how every number in this document
was produced without a Unity installation.
