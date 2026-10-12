using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Tmnf.Game.Pure
{
    /// <summary>
    /// The persistent personal-best table: one best time in milliseconds per
    /// track (keyed by track scene path). Serialized as a flat JSON object —
    /// <c>{"path":timeMs,...}</c> — by a tiny hand-rolled reader/writer so
    /// the store works on every Unity scripting backend without engine
    /// APIs. The runtime half (PersonalBestStore) owns the file location.
    ///
    /// Parsing rules: braces, double-quoted string keys (escaped \" and \\),
    /// integer values, commas. Anything else is a hard error — a corrupted
    /// save is reported, never silently truncated.
    /// </summary>
    public sealed class PersonalBestData
    {
        private readonly Dictionary<string, int> bests =
            new Dictionary<string, int>();

        public int Count => bests.Count;

        /// <summary>The best time for a track, or -1 when none exists yet.</summary>
        public int Get(string trackKey)
        {
            if (trackKey != null && bests.TryGetValue(trackKey, out int value))
            {
                return value;
            }
            return -1;
        }

        /// <summary>
        /// Offers a finished time. Returns true when it improved (or set)
        /// the personal best; a stored, slower result is never downgraded.
        /// </summary>
        public bool Submit(string trackKey, int milliseconds)
        {
            if (trackKey == null || milliseconds < 0)
            {
                return false;
            }
            if (bests.TryGetValue(trackKey, out int current) && current <= milliseconds)
            {
                return false;
            }
            bests[trackKey] = milliseconds;
            return true;
        }

        /// <summary>Directly overwrites an entry (used by tests).</summary>
        public void Set(string trackKey, int milliseconds)
        {
            if (trackKey != null && milliseconds >= 0)
            {
                bests[trackKey] = milliseconds;
            }
        }

        // ------------------------------------------------------------------
        // JSON
        // ------------------------------------------------------------------

        /// <summary>Serializes to the compact JSON object form.</summary>
        public string ToJson()
        {
            StringBuilder sb = new StringBuilder(64 * (bests.Count + 1));
            sb.Append('{');
            bool first = true;
            foreach (KeyValuePair<string, int> pair in bests)
            {
                if (!first)
                {
                    sb.Append(',');
                }
                first = false;
                sb.Append('"');
                AppendEscaped(sb, pair.Key);
                sb.Append('"');
                sb.Append(':');
                sb.Append(pair.Value.ToString(CultureInfo.InvariantCulture));
            }
            sb.Append('}');
            return sb.ToString();
        }

        /// <summary>Parses the JSON object form (throws on malformed input).</summary>
        public static PersonalBestData FromJson(string json)
        {
            PersonalBestData data = new PersonalBestData();
            if (json == null)
            {
                return data;
            }
            int i = 0;
            SkipWhitespace(json, ref i);
            Expect(json, ref i, '{');
            SkipWhitespace(json, ref i);
            if (i < json.Length && json[i] == '}')
            {
                return data;
            }
            while (true)
            {
                SkipWhitespace(json, ref i);
                string key = ParseString(json, ref i);
                SkipWhitespace(json, ref i);
                Expect(json, ref i, ':');
                SkipWhitespace(json, ref i);
                int value = ParseInt(json, ref i);
                if (value < 0)
                {
                    throw new FormatException("personal best: negative time value");
                }
                data.bests[key] = value;
                SkipWhitespace(json, ref i);
                if (i < json.Length && json[i] == ',')
                {
                    i += 1;
                    continue;
                }
                Expect(json, ref i, '}');
                break;
            }
            SkipWhitespace(json, ref i);
            if (i != json.Length)
            {
                throw new FormatException("personal best: trailing content after the object");
            }
            return data;
        }

        private static void SkipWhitespace(string s, ref int i)
        {
            while (i < s.Length && char.IsWhiteSpace(s[i]))
            {
                i += 1;
            }
        }

        private static void Expect(string s, ref int i, char c)
        {
            if (i >= s.Length || s[i] != c)
            {
                throw new FormatException(
                    $"personal best: expected '{c}' at position {i}");
            }
            i += 1;
        }

        private static string ParseString(string s, ref int i)
        {
            Expect(s, ref i, '"');
            StringBuilder sb = new StringBuilder();
            while (i < s.Length)
            {
                char c = s[i];
                i += 1;
                if (c == '"')
                {
                    return sb.ToString();
                }
                if (c == '\\' && i < s.Length)
                {
                    char escaped = s[i];
                    i += 1;
                    switch (escaped)
                    {
                        case '"': sb.Append('"'); break;
                        case '\\': sb.Append('\\'); break;
                        case '/': sb.Append('/'); break;
                        case 'n': sb.Append('\n'); break;
                        case 't': sb.Append('\t'); break;
                        default:
                            throw new FormatException(
                                $"personal best: unsupported escape '\\{escaped}'");
                    }
                    continue;
                }
                sb.Append(c);
            }
            throw new FormatException("personal best: unterminated string");
        }

        private static int ParseInt(string s, ref int i)
        {
            int start = i;
            if (i < s.Length && (s[i] == '-' || s[i] == '+'))
            {
                i += 1;
            }
            while (i < s.Length && char.IsDigit(s[i]))
            {
                i += 1;
            }
            if (i == start)
            {
                throw new FormatException(
                    $"personal best: expected a number at position {start}");
            }
            return int.Parse(s.Substring(start, i - start),
                CultureInfo.InvariantCulture);
        }

        private static void AppendEscaped(StringBuilder sb, string s)
        {
            foreach (char c in s)
            {
                switch (c)
                {
                    case '"': sb.Append("\\\""); break;
                    case '\\': sb.Append("\\\\"); break;
                    case '\n': sb.Append("\\n"); break;
                    case '\t': sb.Append("\\t"); break;
                    default: sb.Append(c); break;
                }
            }
        }
    }
}
