using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// The finish: crossing any of the track's finish triggers completes the
    /// run (once every checkpoint has been collected). A track needs at
    /// least one finish; several are allowed.
    ///
    /// Like checkpoints, the gameplay volume is one designated trigger-only
    /// BoxCollider detected through the custom simulation — it never
    /// physically obstructs the car.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class FinishBlock : MonoBehaviour
    {
        [Tooltip("The one trigger-only BoxCollider that detects the car.")]
        public BoxCollider triggerCollider;

        /// <summary>True when the collider exists, is a trigger and sized.</summary>
        public bool HasValidTrigger => TrackBlocksUtil.IsUsableTrigger(triggerCollider);
    }
}
