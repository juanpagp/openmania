namespace Tmnf.Game
{
    /// <summary>
    /// Cross-scene handoff state: which track the player selected in the
    /// menu, and whether the main menu should open directly on the track
    /// selection list. Kept as plain strings/flags — nothing scene-bound
    /// survives the load.
    /// </summary>
    public static class GameSession
    {
        /// <summary>The selected track's scene path (empty = none).</summary>
        public static string selectedTrackScene = "";

        /// <summary>Display name of the selected track.</summary>
        public static string selectedTrackName = "Track";

        /// <summary>Open the main menu directly on the track list.</summary>
        public static bool showTrackSelect;

        /// <summary>The scene path of the last race (retry support).</summary>
        public static string lastTrackScene = "";
    }
}
