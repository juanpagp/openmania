using System;
using System.Globalization;

namespace Tmnf.Game.Pure
{
    /// <summary>
    /// Race time formatting: <c>mm:ss.mmm</c> for absolute times and signed
    /// <c>+/-mm:ss.mmm</c> for live deltas. All formatting lives here so the
    /// HUD, the results screen and the track list can never disagree.
    /// </summary>
    public static class RaceTimeFormat
    {
        /// <summary>Formats milliseconds as 00:42.381.</summary>
        public static string Time(int milliseconds)
        {
            if (milliseconds < 0)
            {
                milliseconds = 0;
            }
            int minutes = milliseconds / 60000;
            int seconds = (milliseconds % 60000) / 1000;
            int millis = milliseconds % 1000;
            return string.Format(
                CultureInfo.InvariantCulture,
                "{0:00}:{1:00}.{2:000}", minutes, seconds, millis);
        }

        /// <summary>Formats a delta as -00:01.250 / +00:01.250.</summary>
        public static string Delta(int milliseconds)
        {
            string sign = milliseconds < 0 ? "-" : "+";
            int magnitude = Math.Abs(milliseconds);
            int minutes = magnitude / 60000;
            int seconds = (magnitude % 60000) / 1000;
            int millis = magnitude % 1000;
            return string.Format(
                CultureInfo.InvariantCulture,
                "{0}{1:00}:{2:00}.{3:000}", sign, minutes, seconds, millis);
        }

        /// <summary>Placeholder for missing times, e.g. "--:--.---".</summary>
        public const string None = "--:--.---";
    }
}
