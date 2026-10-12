using System.IO;
using Tmnf.Sim;
using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// Loads the vehicle tuning image (the Stadium car snapshot that ships
    /// with the repository as a fixture) and the 128x128 fake-contact mask
    /// the core requires, from StreamingAssets. Both are repository
    /// fixtures, not generated dumps — and if either file ever goes
    /// missing, the pure-data fixture builders regenerate an identical
    /// image in memory so the game still runs (with a console warning).
    /// </summary>
    public static class VehicleFactory
    {
        private const string VehicleFixture =
            "Tmnf.Sim.GoldenData/fixtures/stadium.tmnfvehicle";
        private const string MaskFixture =
            "Tmnf.Sim.GoldenData/fixtures/game-mask.bin";

        /// <summary>
        /// Loads the vehicle image bytes (cached after the first call —
        /// identical for every Sim instance).
        /// </summary>
        public static byte[] LoadVehicleBlob()
        {
            return LoadFixture(VehicleFixture, "vehicle snapshot",
                Tmnf.Sim.Fixtures.TmnfFixtures.StadiumVehicle);
        }

        /// <summary>Loads the fake-contact mask bytes (cached).</summary>
        public static byte[] LoadMask()
        {
            return LoadFixture(MaskFixture, "fake-contact mask",
                Tmnf.Sim.Fixtures.TmnfFixtures.GameMask);
        }

        private static byte[] vehicleCache;
        private static byte[] maskCache;

        private static byte[] LoadFixture(
            string relativePath, string what, System.Func<byte[]> pureFallback)
        {
            if (relativePath == VehicleFixture && vehicleCache != null)
            {
                return vehicleCache;
            }
            if (relativePath == MaskFixture && maskCache != null)
            {
                return maskCache;
            }
            string path = Path.Combine(Application.streamingAssetsPath, relativePath);
            try
            {
                if (File.Exists(path))
                {
                    byte[] data = File.ReadAllBytes(path);
                    Cache(relativePath, data);
                    return data;
                }
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[Tmnf.Game] reading the {what} fixture failed ({e.Message}); " +
                    "falling back to the in-memory fixture builder.");
            }
            Debug.LogWarning(
                $"[Tmnf.Game] {what} fixture missing ({relativePath}); regenerating " +
                "it from pure data. The car uses the same Stadium tuning either way.");
            byte[] built = pureFallback();
            Cache(relativePath, built);
            return built;
        }

        private static void Cache(string relativePath, byte[] data)
        {
            if (relativePath == VehicleFixture)
            {
                vehicleCache = data;
            }
            else if (relativePath == MaskFixture)
            {
                maskCache = data;
            }
        }
    }
}
