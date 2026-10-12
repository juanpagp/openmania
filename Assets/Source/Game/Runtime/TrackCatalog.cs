using System;
using System.Collections.Generic;
using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// The track registration: a ScriptableObject listing every track scene
    /// the game offers. Lives at <c>Assets/Resources/TrackCatalog.asset</c>
    /// so the menu can find it at runtime.
    ///
    /// Adding a track = adding an entry here (assign the scene asset, the
    /// cached display fields refresh via the "OpenMania/Track Catalog" menu
    /// or automatically in OnValidate). No per-track code, no hardcoded
    /// lists: the selection screen enumerates whatever this asset holds.
    /// The live values on the scene's Track component always win at race
    /// time; the cached ones exist so the list can show them without
    /// loading each scene.
    /// </summary>
    [CreateAssetMenu(fileName = "TrackCatalog", menuName = "OpenMania/Track Catalog")]
    public sealed class TrackCatalog : ScriptableObject
    {
        [Serializable]
        public sealed class TrackEntry
        {
            [Tooltip("The track's scene asset (the entry's path is kept in sync).")]
            public UnityEngine.Object scene;

            [Tooltip("Scene path used to load the track at runtime.")]
            public string scenePath = "";

            [Tooltip("Cached display name (refreshed from the scene's Track component).")]
            public string displayName = "Unnamed Track";

            [Tooltip("Cached author (refreshed from the scene's Track component).")]
            public string author = "Unknown";

            [Tooltip("Cached author time in ms, 0 = none (refreshed from the scene).")]
            public int authorTimeMs;

            public bool HasScene => scene != null || !string.IsNullOrEmpty(scenePath);
        }

        public List<TrackEntry> tracks = new List<TrackEntry>();

        /// <summary>Every entry that can actually be loaded.</summary>
        public int Count
        {
            get
            {
                int count = 0;
                foreach (TrackEntry entry in tracks)
                {
                    if (entry.HasScene)
                    {
                        count++;
                    }
                }
                return count;
            }
        }

#if UNITY_EDITOR
        private void OnValidate()
        {
            // Keep the runtime path in sync with the assigned scene asset.
            foreach (TrackEntry entry in tracks)
            {
                if (entry.scene != null)
                {
                    string path = UnityEditor.AssetDatabase.GetAssetPath(entry.scene);
                    if (!string.IsNullOrEmpty(path) && path.EndsWith(".unity")
                        && path != entry.scenePath)
                    {
                        entry.scenePath = path;
                    }
                }
            }
        }
#endif
    }
}
