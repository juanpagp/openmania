using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// The run start: defines where the vehicle spawns and which way it
    /// faces. Every valid track has exactly one.
    ///
    /// The spawn Transform is exposed in the Inspector — point it at an
    /// empty child (created automatically if left empty and a child named
    /// "Spawn" exists). Its +Z axis (the blue arrow) is the direction the
    /// car drives toward; place it slightly above the road surface.
    ///
    /// The block itself needs no mesh and never collides.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class StartBlock : MonoBehaviour
    {
        [Tooltip("Where the car spawns. The transform's +Z (blue) axis is " +
                 "the car's forward direction. Defaults to this object.")]
        public Transform spawnPoint;

        /// <summary>The effective spawn transform (falls back to self).</summary>
        public Transform Spawn => spawnPoint != null ? spawnPoint : transform;

        private void Reset()
        {
            AutoBind();
        }

        private void OnValidate()
        {
            if (spawnPoint == null)
            {
                AutoBind();
            }
        }

        private void AutoBind()
        {
            Transform found = transform.Find("Spawn");
            if (found == null)
            {
                for (int i = 0; i < transform.childCount; i++)
                {
                    if (transform.GetChild(i).name.ToLowerInvariant().Contains("spawn"))
                    {
                        found = transform.GetChild(i);
                        break;
                    }
                }
            }
            if (found != null)
            {
                spawnPoint = found;
            }
        }
    }
}
