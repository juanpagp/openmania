using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Tmnf.Game.EditorTools
{
    /// <summary>
    /// Track catalog maintenance (menu: OpenMania/Track Catalog):
    /// - Refresh metadata: opens every cataloged scene additively (without
    ///   touching your open scenes) and copies the live Track component's
    ///   name/author/author time into the cached entry fields the selection
    ///   screen shows.
    /// - Sync build settings: puts every cataloged scene into the build's
    ///   scene list so track loading works in players.
    /// </summary>
    public static class TrackCatalogEditor
    {
        private const string CatalogPath = "Assets/Resources/TrackCatalog.asset";

        [MenuItem("OpenMania/Track Catalog/Refresh metadata from scenes")]
        public static void RefreshMetadata()
        {
            TrackCatalog catalog = LoadCatalog();
            if (catalog == null)
            {
                return;
            }
            int refreshed = 0;
            foreach (TrackCatalog.TrackEntry entry in catalog.tracks)
            {
                if (string.IsNullOrEmpty(entry.scenePath) || !System.IO.File.Exists(entry.scenePath))
                {
                    if (entry.scene != null)
                    {
                        entry.scenePath = AssetDatabase.GetAssetPath(entry.scene);
                    }
                    if (string.IsNullOrEmpty(entry.scenePath))
                    {
                        Debug.LogWarning($"[OpenMania] catalog entry '{entry.displayName}' " +
                            "has no scene; skipped.");
                        continue;
                    }
                }
                if (!entry.scenePath.EndsWith(".unity"))
                {
                    Debug.LogWarning($"[OpenMania] '{entry.scenePath}' is not a scene; skipped.");
                    continue;
                }

                Scene scene = EditorSceneManager.OpenScene(
                    entry.scenePath, OpenSceneMode.Additive);
                try
                {
                    Track[] tracks = Object.FindObjectsByType<Track>(
                        FindObjectsInactive.Include, FindObjectsSortMode.None);
                    if (tracks.Length != 1)
                    {
                        Debug.LogWarning($"[OpenMania] '{entry.scenePath}' has " +
                            $"{tracks.Length} Track components (need exactly 1); " +
                            "metadata not refreshed.");
                        continue;
                    }
                    entry.displayName = string.IsNullOrWhiteSpace(tracks[0].trackName)
                        ? System.IO.Path.GetFileNameWithoutExtension(entry.scenePath)
                        : tracks[0].trackName;
                    entry.author = tracks[0].author;
                    entry.authorTimeMs = tracks[0].authorTimeMs;
                    refreshed++;
                }
                finally
                {
                    EditorSceneManager.CloseScene(scene, true);
                }
            }
            EditorUtility.SetDirty(catalog);
            AssetDatabase.SaveAssets();
            Debug.Log($"[OpenMania] refreshed metadata for {refreshed} track(s).");
        }

        [MenuItem("OpenMania/Track Catalog/Sync scenes to Build Settings")]
        public static void SyncBuildSettings()
        {
            TrackCatalog catalog = LoadCatalog();
            if (catalog == null)
            {
                return;
            }
            List<EditorBuildSettingsScene> scenes =
                new List<EditorBuildSettingsScene>
                {
                    new EditorBuildSettingsScene("Assets/Scenes/MainMenu.unity", true),
                    new EditorBuildSettingsScene("Assets/Scenes/Race.unity", true),
                };
            foreach (TrackCatalog.TrackEntry entry in catalog.tracks)
            {
                if (string.IsNullOrEmpty(entry.scenePath))
                {
                    continue;
                }
                if (!System.IO.File.Exists(entry.scenePath))
                {
                    Debug.LogWarning($"[OpenMania] '{entry.scenePath}' does not exist; skipped.");
                    continue;
                }
                scenes.Add(new EditorBuildSettingsScene(entry.scenePath, true));
            }
            EditorBuildSettings.scenes = scenes.ToArray();
            Debug.Log($"[OpenMania] build settings now hold {scenes.Count} scene(s) " +
                "(menu, race and every cataloged track).");
        }

        private static TrackCatalog LoadCatalog()
        {
            TrackCatalog catalog = AssetDatabase.LoadAssetAtPath<TrackCatalog>(CatalogPath);
            if (catalog == null)
            {
                Debug.LogError($"[OpenMania] no track catalog at {CatalogPath}. Create one " +
                    "via Create > OpenMania > Track Catalog and save it there.");
            }
            return catalog;
        }
    }
}
