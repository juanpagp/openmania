using System.Collections.Generic;
using Tmnf.Sim;
using Tmnf.Sim.Unity;
using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// The F3 collision debug view: draws the collision shapes the custom
    /// simulation ACTUALLY uses — every ellipsoid of the vehicle's
    /// collision tree (body cyan, wheels yellow, with their real radii and
    /// simulated transforms), the checkpoint/finish trigger volumes
    /// (green/red) and the synthesized reference line — as world-space
    /// wireframes through GL.
    ///
    /// Rendering is completely independent of the visible car model: hide
    /// the vehicle with V (or the inspector flag) and the debug shapes stay.
    /// </summary>
    public sealed class CollisionDebugVisualizer : MonoBehaviour
    {
        private TmnfSim driver;
        private TrackDefinition definition;
        private Material lineMaterial;

        private const int CircleSegments = 24;

        private static readonly Color BodyColor = new Color(0.2f, 0.9f, 1.0f);
        private static readonly Color WheelColor = new Color(1.0f, 0.85f, 0.1f);
        private static readonly Color CheckpointColor = new Color(0.3f, 1.0f, 0.4f);
        private static readonly Color FinishColor = new Color(1.0f, 0.3f, 0.3f);
        private static readonly Color StartColor = new Color(0.4f, 0.6f, 1.0f);
        private static readonly Color LineColor = new Color(0.7f, 0.7f, 0.7f, 0.5f);

        public void Initialize(TmnfSim driver, TrackDefinition definition)
        {
            this.driver = driver;
            this.definition = definition;
            enabled = false; // starts hidden; F3 shows it
        }

        private void OnRenderObject()
        {
            if (driver == null || driver.Sim == null)
            {
                return;
            }
            if (lineMaterial == null)
            {
                lineMaterial = GameMaterials.LineMaterial();
                if (lineMaterial == null)
                {
                    return;
                }
            }
            lineMaterial.SetPass(0);
            GL.PushMatrix();
            GL.Begin(GL.LINES);

            World world = driver.Sim.World;
            GmIso4 carIso = world.Live().RotPosIso();

            // The vehicle tree: root box + every node's box and ellipsoid,
            // through the same flags/localIso composition the narrowphase
            // uses (flags bit 2 = the node's local iso applies).
            DrawBox(ref world.rootBox, ref carIso, new Color(1f, 1f, 1f, 0.35f));
            foreach (int child in world.rootChildren)
            {
                WalkVehicleNode(world, child, ref carIso);
            }

            if (definition != null)
            {
                DrawTriggers(definition.routeSpec.checkpoints, CheckpointColor);
                DrawTriggers(definition.routeSpec.finishes, FinishColor);
                DrawStart(definition.routeSpec.startIso);
                DrawCenterline(definition.routeSpec);
            }

            GL.End();
            GL.PopMatrix();
        }

        private void WalkVehicleNode(World world, int index, ref GmIso4 parent)
        {
            World.VehicleTree node = world.nodes[index];
            GmIso4 iso = node.localIso;
            if ((node.flags & 4u) != 0u)
            {
                iso.Mult(ref parent);
            }
            else
            {
                iso = parent;
            }

            bool isWheel = node.wheel.HasValue;
            Color color = isWheel ? WheelColor : BodyColor;
            DrawBox(ref node.boxAligned, ref iso, new Color(color.r, color.g, color.b, 0.35f));

            if (node.surface != null && node.surface.geom is GmSurfGeomEllipsoid ellipsoid)
            {
                DrawEllipsoid(ref iso, ref ellipsoid.shape.radii, color);
            }
            else if (node.surface != null && node.surface.geom is GmSurfGeomSphere sphere)
            {
                GmVec3 radii = new GmVec3(sphere.shape.radius, sphere.shape.radius, sphere.shape.radius);
                DrawEllipsoid(ref iso, ref radii, color);
            }

            foreach (int child in node.children)
            {
                WalkVehicleNode(world, child, ref iso);
            }
        }

        // ------------------------------------------------------------------
        // primitives (world-space GL lines)
        // ------------------------------------------------------------------

        private void DrawBox(ref GmBoxAligned box, ref GmIso4 iso, Color color)
        {
            Vector3 c = ToUnityPoint(box.center, iso);
            GmMat3 r = default(GmMat3);
            r.m0 = iso.m0; r.m1 = iso.m1; r.m2 = iso.m2;
            r.m3 = iso.m3; r.m4 = iso.m4; r.m5 = iso.m5;
            r.m6 = iso.m6; r.m7 = iso.m7; r.m8 = iso.m8;
            Vector3 ex = ToUnityDir(new GmVec3(box.halfExtent.x, 0, 0), ref r);
            Vector3 ey = ToUnityDir(new GmVec3(0, box.halfExtent.y, 0), ref r);
            Vector3 ez = ToUnityDir(new GmVec3(0, 0, box.halfExtent.z), ref r);

            Vector3 p000 = c - ex - ey - ez;
            Vector3 p100 = c + ex - ey - ez;
            Vector3 p010 = c - ex + ey - ez;
            Vector3 p110 = c + ex + ey - ez;
            Vector3 p001 = c - ex - ey + ez;
            Vector3 p101 = c + ex - ey + ez;
            Vector3 p011 = c - ex + ey + ez;
            Vector3 p111 = c + ex + ey + ez;

            GL.Color(color);
            Line(p000, p100); Line(p100, p110); Line(p110, p010); Line(p010, p000);
            Line(p001, p101); Line(p101, p111); Line(p111, p011); Line(p011, p001);
            Line(p000, p001); Line(p100, p101); Line(p010, p011); Line(p110, p111);
        }

        private void DrawEllipsoid(ref GmIso4 iso, ref GmVec3 radii, Color color)
        {
            Vector3 c = ToUnityPoint(new GmVec3(0, 0, 0), iso);
            GmMat3 r = default(GmMat3);
            r.m0 = iso.m0; r.m1 = iso.m1; r.m2 = iso.m2;
            r.m3 = iso.m3; r.m4 = iso.m4; r.m5 = iso.m5;
            r.m6 = iso.m6; r.m7 = iso.m7; r.m8 = iso.m8;
            // Mirror the radii axes the same way as points (z flips).
            Vector3 rx = ToUnityDir(new GmVec3(radii.x, 0, 0), ref r);
            Vector3 ry = ToUnityDir(new GmVec3(0, radii.y, 0), ref r);
            Vector3 rz = ToUnityDir(new GmVec3(0, 0, radii.z), ref r);

            GL.Color(color);
            Circle(c, rx, ry);
            Circle(c, rx, rz);
            Circle(c, ry, rz);
            // The three semi-axes, for orientation.
            GL.Color(new Color(color.r, color.g, color.b, 0.6f));
            Line(c - rx, c + rx);
            Line(c - ry, c + ry);
            Line(c - rz, c + rz);
        }

        private void Circle(Vector3 center, Vector3 axisA, Vector3 axisB)
        {
            Vector3 prev = center + axisA;
            for (int i = 1; i <= CircleSegments; i++)
            {
                float a = (float)i / CircleSegments * Mathf.PI * 2.0f;
                Vector3 next = center + axisA * Mathf.Cos(a) + axisB * Mathf.Sin(a);
                Line(prev, next);
                prev = next;
            }
        }

        private void DrawTriggers(List<Tmnf.Game.Pure.GameTriggerSpec> triggers, Color color)
        {
            foreach (Tmnf.Game.Pure.GameTriggerSpec trigger in triggers)
            {
                GmIso4 iso = default(GmIso4);
                iso.m0 = trigger.rot[0]; iso.m1 = trigger.rot[1]; iso.m2 = trigger.rot[2];
                iso.m3 = trigger.rot[3]; iso.m4 = trigger.rot[4]; iso.m5 = trigger.rot[5];
                iso.m6 = trigger.rot[6]; iso.m7 = trigger.rot[7]; iso.m8 = trigger.rot[8];
                iso.t0 = trigger.cx; iso.t1 = trigger.cy; iso.t2 = trigger.cz;
                GmBoxAligned box = default(GmBoxAligned);
                box.center = new GmVec3(0, 0, 0);
                box.halfExtent = new GmVec3(trigger.hx, trigger.hy, trigger.hz);
                DrawBox(ref box, ref iso, color);
            }
        }

        private void DrawStart(float[] startIso)
        {
            GmIso4 iso = default(GmIso4);
            iso.m0 = startIso[0]; iso.m1 = startIso[1]; iso.m2 = startIso[2];
            iso.m3 = startIso[3]; iso.m4 = startIso[4]; iso.m5 = startIso[5];
            iso.m6 = startIso[6]; iso.m7 = startIso[7]; iso.m8 = startIso[8];
            iso.t0 = startIso[9]; iso.t1 = startIso[10]; iso.t2 = startIso[11];
            Vector3 origin = ToUnityPoint(new GmVec3(0, 0, 0), iso);
            GmMat3 r = default(GmMat3);
            r.m0 = iso.m0; r.m1 = iso.m1; r.m2 = iso.m2;
            r.m3 = iso.m3; r.m4 = iso.m4; r.m5 = iso.m5;
            r.m6 = iso.m6; r.m7 = iso.m7; r.m8 = iso.m8;
            // The car's forward in TMNF (+Z) shows as the blue axis of the
            // Unity spawn: draw it longer for clarity.
            Vector3 fwd = ToUnityDir(new GmVec3(0, 0, 1), ref r);
            Vector3 up = ToUnityDir(new GmVec3(0, 1, 0), ref r);
            Vector3 right = ToUnityDir(new GmVec3(1, 0, 0), ref r);
            GL.Color(StartColor);
            Line(origin, origin + fwd * 4.0f);
            Line(origin, origin + up * 2.0f);
            Line(origin, origin + right * 2.0f);
        }

        private void DrawCenterline(Tmnf.Game.Pure.GameRouteSpec route)
        {
            GL.Color(LineColor);
            Vector3 prev = Vector3.zero;
            bool has = false;
            foreach (Tmnf.Sim.RouteLoader.TmnfRouteReferencePoint point
                in CenterlinePoints(route))
            {
                Vector3 p = ToUnityPoint(point.position, GmIso4.Identity);
                if (has)
                {
                    Line(prev, p);
                }
                prev = p;
                has = true;
            }
        }

        private static System.Collections.Generic.IEnumerable<Tmnf.Sim.RouteLoader.TmnfRouteReferencePoint>
            CenterlinePoints(Tmnf.Game.Pure.GameRouteSpec route)
        {
            // The pure spec does not keep the synthesized points; rebuild the
            // same waypoint walk for visualization.
            float[] iso = route.startIso;
            yield return new Tmnf.Sim.RouteLoader.TmnfRouteReferencePoint
            {
                position = new GmVec3(iso[9], iso[10], iso[11]),
            };
            foreach (Tmnf.Game.Pure.GameTriggerSpec cp in route.checkpoints)
            {
                yield return new Tmnf.Sim.RouteLoader.TmnfRouteReferencePoint
                {
                    position = new GmVec3(cp.cx, cp.cy, cp.cz),
                };
            }
            if (route.finishes.Count > 0)
            {
                Tmnf.Game.Pure.GameTriggerSpec f = route.finishes[0];
                yield return new Tmnf.Sim.RouteLoader.TmnfRouteReferencePoint
                {
                    position = new GmVec3(f.cx, f.cy, f.cz),
                };
            }
        }

        // ------------------------------------------------------------------
        // conversion helpers
        // ------------------------------------------------------------------

        private static Vector3 ToUnityPoint(GmVec3 p, GmIso4 iso)
        {
            GmVec3 world = p;
            world.MultIso4(ref iso);
            return TmnfCoordinates.ToUnity(world);
        }

        private static Vector3 ToUnityDir(GmVec3 dir, ref GmMat3 rot)
        {
            GmVec3 copy = dir;
            GmVec3 world = default(GmVec3);
            world.SetMultMat3(ref copy, ref rot);
            return TmnfCoordinates.ToUnity(world);
        }

        private static void Line(Vector3 a, Vector3 b)
        {
            GL.Vertex(a);
            GL.Vertex(b);
        }
    }
}
