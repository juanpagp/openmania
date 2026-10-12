using NUnit.Framework;

namespace Tmnf.Game.Tests
{
    /// <summary>
    /// NUnit wrappers around <see cref="TrackPipelineChecks"/> — the same
    /// verification body that also runs inside the standalone dotnet
    /// harness, so the Unity Test Runner (Window > General > Test Runner,
    /// EditMode tab) and the editor-less verification can never diverge.
    /// </summary>
    [TestFixture]
    public class GamePipelineTests
    {
        [Test]
        public void GoldenFacadeHash()
        {
            TrackPipelineChecks.GoldenFacadeHash();
        }

        [Test]
        public void BridgeFloorDrives()
        {
            TrackPipelineChecks.BridgeFloorDrives();
        }

        [Test]
        public void BridgeOneSidedWinding()
        {
            TrackPipelineChecks.BridgeOneSidedWinding();
        }

        [Test]
        public void BridgeScaledMeshBake()
        {
            TrackPipelineChecks.BridgeScaledMeshBake();
        }

        [Test]
        public void BridgeEntryBvhGrid()
        {
            TrackPipelineChecks.BridgeEntryBvhGrid();
        }

        [Test]
        public void RouteCheckpointsAnyOrder()
        {
            TrackPipelineChecks.RouteCheckpointsAnyOrder();
        }

        [Test]
        public void RouteFinishNeedsAllCheckpoints()
        {
            TrackPipelineChecks.RouteFinishNeedsAllCheckpoints();
        }

        [Test]
        public void RouteNoCheckpointsMultipleFinishes()
        {
            TrackPipelineChecks.RouteNoCheckpointsMultipleFinishes();
        }

        [Test]
        public void RouteValidationRejects()
        {
            TrackPipelineChecks.RouteValidationRejects();
        }

        [Test]
        public void RestartFromSameImagesIsClean()
        {
            TrackPipelineChecks.RestartFromSameImagesIsClean();
        }

        [Test]
        public void TimeFormatting()
        {
            TrackPipelineChecks.TimeFormatting();
        }

        [Test]
        public void PersonalBestLogic()
        {
            TrackPipelineChecks.PersonalBestLogic();
        }

        [Test]
        public void SampleTrack01Playable()
        {
            TrackPipelineChecks.SampleTrack01Playable();
        }
    }
}
