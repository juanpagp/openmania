using System;
using System.IO;
using Tmnf.Game.Pure;
using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// The persistent personal-best table: one best time per track (keyed by
    /// scene path), stored as JSON in <c>persistentDataPath/openmania-pb.json</c>.
    /// Loaded once per run; saved whenever a finish improves a time.
    /// </summary>
    public static class PersonalBestStore
    {
        private static PersonalBestData data;
        private static string path;

        /// <summary>The best time for a track in ms, or -1 when none.</summary>
        public static int Get(string trackKey)
        {
            Load();
            return data.Get(trackKey);
        }

        /// <summary>Offers a finished time; saves when it improves the best.</summary>
        public static bool Submit(string trackKey, int milliseconds)
        {
            Load();
            if (!data.Submit(trackKey, milliseconds))
            {
                return false;
            }
            Save();
            return true;
        }

        private static void Load()
        {
            if (data != null)
            {
                return;
            }
            path = Path.Combine(Application.persistentDataPath, "openmania-pb.json");
            data = new PersonalBestData();
            try
            {
                if (File.Exists(path))
                {
                    data = PersonalBestData.FromJson(File.ReadAllText(path));
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning(
                    $"[Tmnf.Game] could not read the personal-best file ({e.Message}); " +
                    "starting with an empty table. The corrupted file was not overwritten " +
                    "until the next improvement is saved.");
                data = new PersonalBestData();
            }
        }

        private static void Save()
        {
            try
            {
                string directory = Path.GetDirectoryName(path);
                if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                {
                    Directory.CreateDirectory(directory);
                }
                File.WriteAllText(path, data.ToJson());
            }
            catch (Exception e)
            {
                Debug.LogWarning(
                    $"[Tmnf.Game] could not save the personal-best file ({e.Message}).");
            }
        }
    }
}
