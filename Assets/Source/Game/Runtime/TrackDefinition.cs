using System.Collections.Generic;
using Tmnf.Game.Pure;

namespace Tmnf.Game
{
    /// <summary>
    /// Everything extracted from one loaded track scene, ready to be turned
    /// into simulation images: the pure track/route specs, the metadata and
    /// the diagnostics collected on the way. Produced by
    /// <see cref="TrackSceneExtractor"/>; the game layer only consumes it.
    /// </summary>
    public sealed class TrackDefinition
    {
        /// <summary>Track metadata component (name, author, author time).</summary>
        public Track metadata;

        /// <summary>Loaded scene path (personal-best key).</summary>
        public string scenePath;

        /// <summary>Collidable geometry (meshes, materials), TMNF space.</summary>
        public GameTrackSpec trackSpec;

        /// <summary>Start, checkpoints, finishes, TMNF space.</summary>
        public GameRouteSpec routeSpec;

        /// <summary>Validation errors — a non-empty list means the race
        /// must not start. Filled by the extractor's checks.</summary>
        public List<string> errors = new List<string>();

        /// <summary>Non-fatal diagnostics (skipped meshes, flipped winding ...).</summary>
        public List<string> warnings = new List<string>();

        public bool Valid => errors.Count == 0;
    }
}
