using Tmnf.Game;
using UnityEditor;
using UnityEngine;

namespace Tmnf.Game.EditorTools
{
    /// <summary>
    /// Scene-view authoring aids: Start blocks show their spawn arrow (the
    /// blue +Z axis the car drives toward), checkpoint/finish triggers show
    /// their volumes, and TrackGeometry roots are outlined — making a track
    /// readable at a glance while editing.
    /// </summary>
    public static class TrackGizmos
    {
        private static readonly Color StartColor = new Color(0.35f, 0.65f, 1.0f);
        private static readonly Color CheckpointColor = new Color(0.35f, 1.0f, 0.45f);
        private static readonly Color FinishColor = new Color(1.0f, 0.35f, 0.35f);
        private static readonly Color GeometryColor = new Color(0.35f, 1.0f, 1.0f, 0.6f);

        [DrawGizmo(GizmoType.InSelectionHierarchy | GizmoType.NotInSelectionHierarchy
            | GizmoType.Pickable)]
        private static void DrawStart(StartBlock block, GizmoType type)
        {
            Transform spawn = block.Spawn;
            Gizmos.color = StartColor;
            Gizmos.matrix = spawn.localToWorldMatrix;
            Gizmos.DrawWireCube(Vector3.zero, new Vector3(2.2f, 1.1f, 4.4f));
            Vector3 forward = Vector3.forward * 4.0f; // the +Z the car drives toward
            Gizmos.DrawLine(Vector3.zero, forward);
            // An arrow head so the facing is unambiguous.
            Gizmos.DrawLine(forward, forward + new Vector3(0.3f, 0f, -0.6f));
            Gizmos.DrawLine(forward, forward + new Vector3(-0.3f, 0f, -0.6f));
            Gizmos.DrawLine(forward, forward + new Vector3(0f, 0.3f, -0.6f));
            Gizmos.matrix = Matrix4x4.identity;
        }

        [DrawGizmo(GizmoType.InSelectionHierarchy | GizmoType.NotInSelectionHierarchy
            | GizmoType.Pickable)]
        private static void DrawCheckpoint(CheckpointBlock block, GizmoType type)
        {
            DrawTrigger(block.triggerCollider, CheckpointColor);
        }

        [DrawGizmo(GizmoType.InSelectionHierarchy | GizmoType.NotInSelectionHierarchy
            | GizmoType.Pickable)]
        private static void DrawFinish(FinishBlock block, GizmoType type)
        {
            DrawTrigger(block.triggerCollider, FinishColor);
        }

        private static void DrawTrigger(BoxCollider collider, Color color)
        {
            if (collider == null)
            {
                return;
            }
            Gizmos.color = color;
            Gizmos.matrix = collider.transform.localToWorldMatrix;
            Gizmos.DrawWireCube(collider.center, collider.size);
            Gizmos.matrix = Matrix4x4.identity;
        }

        [DrawGizmo(GizmoType.InSelectionHierarchy | GizmoType.NotInSelectionHierarchy)]
        private static void DrawGeometry(TrackGeometry geometry, GizmoType type)
        {
            Gizmos.color = GeometryColor;
            Gizmos.matrix = geometry.transform.localToWorldMatrix;
            Gizmos.DrawWireCube(Vector3.zero, Vector3.one);
            Gizmos.matrix = Matrix4x4.identity;
        }
    }
}
