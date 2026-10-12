using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// A checkpoint the vehicle must pass through (in any order — all of the
    /// track's checkpoints must be collected before a finish counts).
    ///
    /// The gameplay volume is one designated trigger-only BoxCollider: it
    /// detects the car through the custom simulation's trigger path and
    /// never physically obstructs it. Build any visual gate you like around
    /// it — visuals are entirely separate from the trigger.
    ///
    /// A track may have zero checkpoints.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class CheckpointBlock : MonoBehaviour
    {
        [Tooltip("The one trigger-only BoxCollider that detects the car.")]
        public BoxCollider triggerCollider;

        /// <summary>True when the collider exists, is a trigger and sized.</summary>
        public bool HasValidTrigger => TrackBlocksUtil.IsUsableTrigger(triggerCollider);
    }
}
