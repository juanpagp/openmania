using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// The root of a track: attach to the top-level GameObject of a track
    /// scene. Everything under this object is the track's content — mesh
    /// geometry (see <see cref="TrackGeometry"/>) and the special gameplay
    /// objects (<see cref="StartBlock"/>, <see cref="CheckpointBlock"/>,
    /// <see cref="FinishBlock"/>).
    ///
    /// A track scene needs exactly one active Track component. Its metadata
    /// (name, author, author time) is what the track selection screen and
    /// the race HUD display; the scene's path is the personal-best key.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class Track : MonoBehaviour
    {
        [Tooltip("Display name shown in the track list and the results screen.")]
        public string trackName = "Unnamed Track";

        [Tooltip("Author shown in the track list.")]
        public string author = "Unknown";

        [Tooltip("Author time in milliseconds (0 = not set). The benchmark " +
                 "time established by the track's validation run.")]
        public int authorTimeMs;

        [Tooltip("Optional free-form comment shown on the results screen.")]
        public string comment = "";

        [Tooltip("When true, every Renderer under this Track is assigned a " +
                 "simple URP material when the race loads — handy for quick " +
                 "greybox tracks. Turn off for tracks that bring their own " +
                 "materials.")]
        public bool applyDefaultMaterialOnLoad = true;

        /// <summary>The author time, or -1 when none is set.</summary>
        public int AuthorTimeOrNone => authorTimeMs > 0 ? authorTimeMs : -1;
    }
}
