using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using NUnit.Framework;

namespace Tmnf.Sim.Tests
{
    /// <summary>
    /// The forbidden-API and float-literal scan over the CORE sources
    /// (the same rules as scripts/forbidden_scan.py):
    ///
    /// - The core may never reference Unity types or fast-math entry
    ///   points: no UnityEngine, Mathf, MathF, System.Numerics,
    ///   Unity.Mathematics, Burst, FMA. (System.Math with explicit double
    ///   casts is the sanctioned sqrt/abs route.)
    /// - Decimal literals in float contexts must carry the f suffix; the
    ///   f64 single-rounding blocks carry an explicit d suffix (the scan
    ///   enforces that convention: an unsuffixed decimal literal on a line
    ///   without a double token is a violation). Fp/Libm.cs is the
    ///   pure-double fdlibm port and is exempt as a file.
    ///
    /// In the .NET run this scans the real sources. Under Unity Play Mode
    /// the source tree is not reachable from the sandbox, so the scan is
    /// skipped there (CI runs the .NET lane, which is authoritative).
    /// </summary>
    [TestFixture]
    public class ForbiddenApiScan
    {
        private static readonly string[] Forbidden =
        {
            "UnityEngine",
            "Mathf.",
            "MathF.",
            "System.Numerics",
            "Unity.Mathematics",
            "Unity.Burst",
            "FusedMultiplyAdd",
            "Vector3",
            "Quaternion",
        };

        private static readonly HashSet<string> FileAllowlist = new HashSet<string>
        {
            "Fp/Libm.cs", // pure-double fdlibm port: every literal is a double
        };

        // f64-block mirrors that carry no double token on their own line.
        // The convention is to write them with the d suffix instead, so this
        // list should stay EMPTY — it exists as the documented escape hatch.
        private static readonly HashSet<string> LineAllowlist = new HashSet<string>();

        private static readonly Regex LiteralRe = new Regex(
            @"\b\d+\.\d+(?:[eE][+-]?\d+)?(?![A-Za-z0-9_])",
            RegexOptions.Compiled);

        [Test]
        public void CoreSourcesAreClean()
        {
            string coreDir = FindCoreDir();
            if (coreDir == null)
            {
                Assert.Ignore("core sources not reachable from this sandbox (run the .NET/CI lane)");
            }

            var violations = new List<string>();
            foreach (string file in Directory.GetFiles(coreDir, "*.cs", SearchOption.AllDirectories))
            {
                string rel = RelPath(coreDir, file);
                if (FileAllowlist.Contains(rel))
                {
                    continue;
                }
                // generated build artifacts (obj/bin) are never policy-scanned
                if (rel.StartsWith("obj/", StringComparison.Ordinal)
                    || rel.StartsWith("bin/", StringComparison.Ordinal))
                {
                    continue;
                }
                string[] lines = File.ReadAllLines(file);
                for (int n = 0; n < lines.Length; n++)
                {
                    string code = StripComment(lines[n]);
                    if (code.Trim().Length == 0)
                    {
                        continue;
                    }
                    foreach (string tok in Forbidden)
                    {
                        if (code.Contains(tok))
                        {
                            violations.Add($"{rel}:{n + 1}: forbidden '{tok}'");
                        }
                    }
                    if (LineAllowlist.Contains(code.Trim()))
                    {
                        continue;
                    }
                    if (code.Contains("double") || code.Contains("Double") || code.Contains("X87"))
                    {
                        continue;
                    }
                    Match m = LiteralRe.Match(code);
                    if (m.Success)
                    {
                        violations.Add($"{rel}:{n + 1}: unsuffixed decimal literal '{m.Value}'");
                    }
                }
            }
            Assert.That(violations, Is.Empty,
                "core FP-policy violations (" + violations.Count + "):\n" + string.Join("\n", violations));
        }

        private static string StripComment(string line)
        {
            int idx = line.IndexOf("//", StringComparison.Ordinal);
            return idx < 0 ? line : line.Substring(0, idx);
        }

        private static string RelPath(string root, string file)
        {
            string full = Path.GetFullPath(file);
            string r = Path.GetFullPath(root).TrimEnd(Path.DirectorySeparatorChar) +
                       Path.DirectorySeparatorChar;
            return full.StartsWith(r, StringComparison.Ordinal)
                ? full.Substring(r.Length).Replace('\\', '/')
                : full;
        }

        private static string FindCoreDir()
        {
            // .NET: .../Assets/Tests/Tmnf.Sim.Tests/bin/Release/net8.0 ->
            // walk up until a sibling "Source/Core" appears.
            string dir = AppContext.BaseDirectory;
            for (int i = 0; i < 8 && dir != null; i++)
            {
                string candidate = Path.Combine(dir, "Source", "Core");
                if (Directory.Exists(candidate))
                {
                    return candidate;
                }
                string candidate2 = Path.Combine(dir, "Assets", "Source", "Core");
                if (Directory.Exists(candidate2))
                {
                    return candidate2;
                }
                dir = Path.GetDirectoryName(dir);
            }
            return null;
        }
    }
}
