namespace Tmnf.Sim
{
    /// <summary>CPlugSurface physical material ids — the 31 shared materials.</summary>
    public static class SurfaceMaterials
    {
        public const uint TmnfSurfaceConcrete = 0;
        public const uint TmnfSurfacePavement = 1;
        public const uint TmnfSurfaceGrass = 2;
        public const uint TmnfSurfaceIce = 3;
        public const uint TmnfSurfaceMetal = 4;
        public const uint TmnfSurfaceSand = 5;
        public const uint TmnfSurfaceDirt = 6;
        public const uint TmnfSurfaceTurbo = 7;
        public const uint TmnfSurfaceDirtRoad = 8;
        public const uint TmnfSurfaceRubber = 9;
        public const uint TmnfSurfaceSlidingRubber = 10;
        public const uint TmnfSurfaceTest = 11;
        public const uint TmnfSurfaceRock = 12;
        public const uint TmnfSurfaceWater = 13;
        public const uint TmnfSurfaceWood = 14;
        public const uint TmnfSurfaceDanger = 15;
        public const uint TmnfSurfaceAsphalt = 16;
        public const uint TmnfSurfaceWetDirtRoad = 17;
        public const uint TmnfSurfaceWetAsphalt = 18;
        public const uint TmnfSurfaceWetPavement = 19;
        public const uint TmnfSurfaceWetGrass = 20;
        public const uint TmnfSurfaceSnow = 21;
        public const uint TmnfSurfaceResonantMetal = 22;
        public const uint TmnfSurfaceGolfBall = 23;
        public const uint TmnfSurfaceGolfWall = 24;
        public const uint TmnfSurfaceGolfGround = 25;
        public const uint TmnfSurfaceTurbo2 = 26;
        public const uint TmnfSurfaceBumper = 27;
        public const uint TmnfSurfaceNotCollidable = 28;
        public const uint TmnfSurfaceFreeWheeling = 29;
        public const uint TmnfSurfaceTurboRoulette = 30;
        public const uint TmnfSurfaceMaterialCount = 31;

        /// <summary>Upper bound on material-manager entries a vehicle carries.</summary>
        public const uint TmnfMaxGroundMaterials = 32;

        /// <summary>The 31 reflection names.</summary>
        public static readonly string[] TmnfSurfaceMaterialNames =
        {
            "Concrete", "Pavement", "Grass", "Ice", "Metal", "Sand", "Dirt",
            "Turbo", "DirtRoad", "Rubber", "SlidingRubber", "Test", "Rock",
            "Water", "Wood", "Danger", "Asphalt", "WetDirtRoad", "WetAsphalt",
            "WetPavement", "WetGrass", "Snow", "ResonantMetal", "GolfBall",
            "GolfWall", "GolfGround", "Turbo2", "Bumper", "NotCollidable",
            "FreeWheeling", "TurboRoulette",
        };

        /// <summary>The reflection name of a material id (panics out of range).</summary>
        public static string TmnfSurfaceMaterialName(uint materialId)
        {
            if (materialId >= TmnfSurfaceMaterialCount)
            {
                throw new System.IndexOutOfRangeException("tmnf: surface material id out of range");
            }
            return TmnfSurfaceMaterialNames[materialId];
        }
    }
}
