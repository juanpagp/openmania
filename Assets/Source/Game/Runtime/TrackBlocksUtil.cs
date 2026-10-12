using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>Shared helpers for the gameplay block components.</summary>
    public static class TrackBlocksUtil
    {
        /// <summary>
        /// A trigger collider is usable when it exists, is marked trigger
        /// (so it can never physically block the car — the custom simulation
        /// reads it as a pure volume) and has a positive size.
        /// </summary>
        public static bool IsUsableTrigger(BoxCollider collider)
        {
            return collider != null
                && collider.enabled
                && collider.isTrigger
                && collider.size.x > 0.0f
                && collider.size.y > 0.0f
                && collider.size.z > 0.0f;
        }
    }
}
