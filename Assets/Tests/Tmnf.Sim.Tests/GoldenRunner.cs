using System;
using System.Collections.Generic;
using System.IO;
using Tmnf.Sim;

namespace Tmnf.Sim.Tests
{
    /// <summary>
    /// The TGN1 golden-file reader and the bit-exact comparison with
    /// first-divergence diagnostics. The data lives once, checked in at
    /// Assets/StreamingAssets/Tmnf.Sim.GoldenData (copied into the test
    /// output; Unity reads StreamingAssets directly).
    /// </summary>
    public static class GoldenRunner
    {
        public static string GoldenRoot()
        {
            // .NET: copied to output/GoldenData. Unity: StreamingAssets.
            string output = Path.Combine(AppContext.BaseDirectory, "GoldenData");
            if (Directory.Exists(output))
            {
                return output;
            }
            string cwdRoot = Path.Combine(Directory.GetCurrentDirectory(), "GoldenData");
            if (Directory.Exists(cwdRoot))
            {
                return cwdRoot;
            }
            string assets = Path.Combine(Directory.GetCurrentDirectory(),
                "..", "..", "..", "..", "StreamingAssets", "Tmnf.Sim.GoldenData");
            if (Directory.Exists(assets))
            {
                return assets;
            }
            throw new InvalidOperationException(
                $"golden data not found (looked in {output}, {cwdRoot}, {assets})");
        }

        public static string GoldenPath(string relative)
        {
            return Path.Combine(GoldenRoot(), relative.Replace('/', Path.DirectorySeparatorChar));
        }

        /// <summary>A (input, output) row pair.</summary>
        public struct Row
        {
            public byte[] Input;
            public byte[] Output;
        }

        /// <summary>Reads every row of a .tgn suite file.</summary>
        public static List<Row> ReadRows(string relativePath)
        {
            byte[] file = File.ReadAllBytes(GoldenPath(relativePath));
            int pos = 0;
            if (file[0] != (byte)'T' || file[1] != (byte)'G' || file[2] != (byte)'N' || file[3] != (byte)'1')
            {
                throw new InvalidDataException($"bad magic in {relativePath}");
            }
            pos = 4;
            uint nameLen = BitsUtil.GetU32(file, pos); pos += 4;
            pos += (int)nameLen;
            ulong rowCount = BitsUtil.GetU64(file, pos); pos += 8;
            var rows = new List<Row>((int)rowCount);
            for (ulong i = 0; i < rowCount; i++)
            {
                uint inLen = BitsUtil.GetU32(file, pos); pos += 4;
                byte[] input = new byte[inLen];
                Buffer.BlockCopy(file, pos, input, 0, (int)inLen); pos += (int)inLen;
                uint outLen = BitsUtil.GetU32(file, pos); pos += 4;
                byte[] output = new byte[outLen];
                Buffer.BlockCopy(file, pos, output, 0, (int)outLen); pos += (int)outLen;
                rows.Add(new Row { Input = input, Output = output });
            }
            return rows;
        }

        /// <summary>
        /// Runs a suite: for each row, the runner produces the output bytes
        /// and they are compared raw — never with tolerance. The first
        /// mismatch reports the row index, byte offset and both hex bytes.
        /// </summary>
        public static int RunSuite(string relativePath, Func<byte[], byte[]> runRow)
        {
            List<Row> rows = ReadRows(relativePath);
            for (int i = 0; i < rows.Count; i++)
            {
                byte[] actual;
                try
                {
                    actual = runRow(rows[i].Input);
                }
                catch (Exception ex)
                {
                    throw new AssertionException(
                        $"{relativePath} row {i}: runner threw: {ex.GetType().Name}: {ex.Message}\n" +
                        $"{ex.StackTrace}");
                }
                byte[] expected = rows[i].Output;
                if (actual == null)
                {
                    throw new AssertionException($"{relativePath} row {i}: runner returned null");
                }
                if (actual.Length != expected.Length)
                {
                    throw new AssertionException(
                        $"{relativePath} row {i}: output length {actual.Length} != expected {expected.Length}");
                }
                for (int b = 0; b < expected.Length; b++)
                {
                    if (actual[b] != expected[b])
                    {
                        int word = b & ~3;
                        string ctxE = Hex(expected, word);
                        string ctxA = Hex(actual, word);
                        throw new AssertionException(
                            $"{relativePath} row {i}: first divergence at byte {b}: " +
                            $"expected 0x{expected[b]:X2}, actual 0x{actual[b]:X2} " +
                            $"(word @{word}: expected 0x{ctxE}, actual 0x{ctxA})");
                    }
                }
            }
            return rows.Count;
        }

        private static string Hex(byte[] b, int o)
        {
            if (o + 4 > b.Length)
            {
                return "(past end)";
            }
            return $"{BitsUtil.GetU32(b, o):X8}";
        }

        /// <summary>
        /// Runs a suite where the runner may panic by contract (panic-marker
        /// rows): the runner signals a panic by throwing
        /// <see cref="IndexOutOfRangeException"/> or the domain panic; the
        /// row's first output byte carries the marker.
        /// </summary>
        public static int RunSuitePanicAware(string relativePath, Action<byte[], List<byte>> runRow)
        {
            return RunSuite(relativePath, input =>
            {
                var output = new List<byte>(4096);
                try
                {
                    runRow(input, output);
                }
                catch (Exception ex) when (
                    ex is IndexOutOfRangeException
                    || ex is InvalidOperationException
                    || ex is OverflowException)
                {
                    if (IsPanicException(ex))
                    {
                        output.Insert(0, 1); // panic marker
                        return output.ToArray();
                    }
                    // The panic itself is the observable behavior: record it.
                    output.Clear();
                    output.Add(1); // panic marker
                    return output.ToArray();
                }
                output.Insert(0, 0); // success marker
                return output.ToArray();
            });
        }

        private static bool IsPanicException(Exception ex)
        {
            // The port's panics surface as IndexOutOfRange (OOB index) or
            // InvalidOperationException (explicit tmnf: guards).
            return ex is IndexOutOfRangeException || ex is InvalidOperationException;
        }
    }

    /// <summary>Minimal assertion helper (avoids the NUnit dependency in helpers).</summary>
    public class AssertionException : Exception
    {
        public AssertionException(string message) : base(message) { }
    }
}
