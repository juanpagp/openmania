using System;

namespace Tmnf.Sim
{
    /// <summary>The race layer: checkpoints, laps, off-track, respawn.</summary>
    public static class Race
    {
        public const uint TmnfRaceTickMs = 10;
        public const uint TmnfRaceProjectionWindowSegments = 32;
        public const uint TmnfRaceTeleportDistanceMeters = 32;
        public const uint TmnfRaceMaxCheckpoints = 63;
        public const uint TmnfRaceFinishContactBit = 63;

        public const uint TmnfMaterialGrass = SurfaceMaterials.TmnfSurfaceGrass;
        public const uint TmnfMaterialWetGrass = SurfaceMaterials.TmnfSurfaceWetGrass;

        public const float TmnfRaceCorridorWidthFactor = 3.0f;
        public const float TmnfRaceCorridorWidthFloorMeters = 28.0f;
        public const float TmnfRaceCorridorAboveMeters = 44.0f;
        public const float TmnfRaceCorridorBelowMeters = 20.0f;

        /// <summary>The race bookkeeping state.</summary>
        [Serializable]
        public struct TmnfRaceState
        {
            /// <summary>bit i: checkpoint i taken this lap.</summary>
            public ulong visitedCheckpoints;
            /// <summary>low bits: checkpoints; finish i uses bit 63-i.</summary>
            public ulong triggerContacts;
            public uint visitedCount;
            public uint completedLaps;
            public uint passedRaceCheckpoints;
            public uint elapsedTicks;
            public uint finishTimeMs;
            public uint projectionSegment;
            public uint centerlineSegment;
            public uint offTrackTicks;
            public uint stuckTicks;
            public float arcLength;
            /// <summary>3D distance to the centerline.</summary>
            public float lateralOffset;
            public float halfWidth;
            /// <summary>XZ distance to the projected point.</summary>
            public float corridorLateral;
            /// <summary>car y minus projected point y.</summary>
            public float corridorVertical;
            public float unwrappedProgress;
            public float bestProgress;
            public float previousProgress;
            public GmIso4 previousCarTransform;
            /// <summary>Where a respawn places the car.</summary>
            public GmIso4 respawnLocation;
            public byte finished;
            /// <summary>Progress frozen this tick.</summary>
            public byte outsideCorridor;
            /// <summary>A respawnable checkpoint was passed.</summary>
            public byte respawnAvailable;
            public byte reserved;

            public static TmnfRaceState Create()
            {
                TmnfRaceState s = default(TmnfRaceState);
                s.previousCarTransform = GmIso4.Identity;
                s.respawnLocation = GmIso4.Identity;
                return s;
            }
        }

        /// <summary>TmnfRaceStepResult.</summary>
        [Serializable]
        public struct TmnfRaceStepResult
        {
            public byte checkpointAccepted;
            public byte checkpointRepeated;
            public byte lapCompleted;
            public byte finished;
            public uint checkpointIndex;
            public uint raceTimeMs;
        }

        // ------------------------------------------------------------------
        // validation
        // ------------------------------------------------------------------

        private static void ValidateRoute(RouteLoader.TmnfRoute route)
        {
            RouteLoader.TmnfRouteMetadata metadata = route.metadata;
            if (metadata.lapCount == 0
                || metadata.finishCount == 0
                || (ulong)metadata.finishCount + metadata.checkpointCount > 64
                || metadata.checkpointCount > TmnfRaceMaxCheckpoints
                || metadata.referenceCount != metadata.checkpointCount + 2
                || metadata.centerlineCount < 2)
            {
                throw new InvalidOperationException("tmnf race: route metadata is inconsistent");
            }
            if (((metadata.flags & RouteLoader.TmnfRouteMultilap) != 0) != (metadata.lapCount > 1))
            {
                throw new InvalidOperationException("tmnf race: route multilap flag is inconsistent");
            }
            ulong expected = ((ulong)metadata.checkpointCount + 1) * metadata.lapCount;
            if (expected > uint.MaxValue || metadata.totalRaceCheckpoints != (uint)expected)
            {
                throw new InvalidOperationException("tmnf race: route total checkpoint count is inconsistent");
            }
        }

        // ------------------------------------------------------------------
        // trigger volume as a twelve-face box mesh
        // ------------------------------------------------------------------

        /// <summary>The face's AABB from its three vertices (face_box).</summary>
        private static GmBoxAligned FaceBox(ref GmVec3 a, ref GmVec3 b, ref GmVec3 c)
        {
            GmVec3 low = a;
            GmVec3 high = a;
            GmVec3[] points = { b, c };
            foreach (GmVec3 p in points)
            {
                if (p.x < low.x) low.x = p.x;
                if (p.y < low.y) low.y = p.y;
                if (p.z < low.z) low.z = p.z;
                if (p.x > high.x) high.x = p.x;
                if (p.y > high.y) high.y = p.y;
                if (p.z > high.z) high.z = p.z;
            }
            GmBoxAligned r = default(GmBoxAligned);
            r.center = new GmVec3(
                (float)(double)(0.5f * ((float)(double)(low.x + high.x))),
                (float)(double)(0.5f * ((float)(double)(low.y + high.y))),
                (float)(double)(0.5f * ((float)(double)(low.z + high.z))));
            r.halfExtent = new GmVec3(
                (float)(double)(0.5f * ((float)(double)(high.x - low.x))),
                (float)(double)(0.5f * ((float)(double)(high.y - low.y))),
                (float)(double)(0.5f * ((float)(double)(high.z - low.z))));
            return r;
        }

        /// <summary>add_face: one box-face triangle, wound along `outward`.</summary>
        private static void AddFace(
            GmSurfMeshFace[] faces, GmSurfMeshNode[] nodes, GmVec3[] vertices,
            int index, uint v0, uint v1, uint v2, ref GmVec3 outward)
        {
            GmVec3 a = vertices[v0];
            GmVec3 b = vertices[v1];
            GmVec3 c = vertices[v2];
            GmVec3 e1 = new GmVec3((float)(double)(b.x - a.x), (float)(double)(b.y - a.y), (float)(double)(b.z - a.z));
            GmVec3 e2 = new GmVec3((float)(double)(c.x - a.x), (float)(double)(c.y - a.y), (float)(double)(c.z - a.z));
            GmVec3 n = new GmVec3(
                (float)(double)((float)(double)(e1.y * e2.z) - (float)(double)(e1.z * e2.y)),
                (float)(double)((float)(double)(e1.z * e2.x) - (float)(double)(e1.x * e2.z)),
                (float)(double)((float)(double)(e1.x * e2.y) - (float)(double)(e1.y * e2.x)));
            uint w1, w2;
            if ((float)(double)((float)(double)(((float)(double)(n.x * outward.x)) + ((float)(double)(n.y * outward.y))) + ((float)(double)(n.z * outward.z))) < 0.0f)
            {
                w1 = v2;
                w2 = v1;
            }
            else
            {
                w1 = v1;
                w2 = v2;
            }
            GmSurfMeshFace face = default(GmSurfMeshFace); // memset(face, 0, ...)
            face.normal = outward;
            face.vertex0 = v0;
            face.vertex1 = w1;
            face.vertex2 = w2;
            face.materialIndex = 0;
            faces[index] = face;
            GmSurfMeshNode node = default(GmSurfMeshNode);
            node.skipCount = 1;
            node.boxAligned = FaceBox(ref vertices[v0], ref vertices[w1], ref vertices[w2]);
            node.faceIndex = (uint)index;
            nodes[index] = node;
        }

        /// <summary>build_trigger_mesh: the trigger box as a 12-triangle mesh.</summary>
        private static CPlugSurface BuildTriggerMesh(ref GmBoxAligned boxAligned)
        {
            if (!(boxAligned.halfExtent.x > 0.0f)
                || !(boxAligned.halfExtent.y > 0.0f)
                || !(boxAligned.halfExtent.z > 0.0f))
            {
                throw new InvalidOperationException("tmnf race: trigger box has a non-positive extent");
            }
            GmVec3[] vertices = new GmVec3[8];
            for (uint i = 0; i < 8; i++)
            {
                vertices[i].x = (float)(double)(boxAligned.center.x
                    + ((i & 1) != 0 ? boxAligned.halfExtent.x : -boxAligned.halfExtent.x));
                vertices[i].y = (float)(double)(boxAligned.center.y
                    + ((i & 2) != 0 ? boxAligned.halfExtent.y : -boxAligned.halfExtent.y));
                vertices[i].z = (float)(double)(boxAligned.center.z
                    + ((i & 4) != 0 ? boxAligned.halfExtent.z : -boxAligned.halfExtent.z));
            }
            GmSurfMeshFace[] faces = new GmSurfMeshFace[12];
            GmSurfMeshNode[] nodes = new GmSurfMeshNode[12];
            int face = 0;
            for (uint axis = 0; axis < 3; axis++)
            {
                uint bit = 1u << (int)axis;
                for (uint side = 0; side < 2; side++)
                {
                    GmVec3 outward = GmVec3.Zero;
                    float sign = side != 0 ? 1.0f : -1.0f;
                    if (axis == 0) outward.x = sign;
                    else if (axis == 1) outward.y = sign;
                    else outward.z = sign;
                    uint[] corners = new uint[4];
                    int count = 0;
                    for (uint v = 0; v < 8; v++)
                    {
                        if (((v & bit) != 0) == (side != 0))
                        {
                            corners[count] = v;
                            count += 1;
                        }
                    }
                    // corners in increasing index order: 0-1-3 and 0-3-2 tile
                    // the quad over the two remaining axes.
                    AddFace(faces, nodes, vertices, face, corners[0], corners[1], corners[3], ref outward);
                    face += 1;
                    AddFace(faces, nodes, vertices, face, corners[0], corners[3], corners[2], ref outward);
                    face += 1;
                }
            }
            GmSurfMesh mesh = new GmSurfMesh();
            mesh.@base = default(GmSurf);
            mesh.@base.surfType = GmSurfTypes.GmSurfMesh;
            mesh.vertexCount = 8;
            mesh.vertices = vertices;
            mesh.faceCount = 12;
            mesh.faces = faces;
            mesh.nodeCount = 12;
            mesh.nodes = nodes;
            CPlugSurface surface = new CPlugSurface();
            surface.geom = new GmSurfGeomMesh { mesh = mesh };
            surface.materialIds = new byte[] { 0 };
            surface.materialCount = 1;
            return surface;
        }

        // ------------------------------------------------------------------
        // car subtrees against the trigger
        // ------------------------------------------------------------------

        private const int TriggerTreeStackCapacity = 16;

        /// <summary>car_tree_contact: the car tree against the trigger surface.</summary>
        private static bool CarTreeContact(
            CPlugTree[] carTrees,
            int carTree,
            ref GmIso4 carWorldTransform,
            ref GmBoxAligned triggerWorldBox,
            CPlugSurface mesh,
            ref GmIso4 triggerIso,
            CHmsCollisionBuffer buffer,
            Narrowphase.CollisionShapeDispatch runtime)
        {
            var stackIso = new GmIso4[TriggerTreeStackCapacity];
            var stackIndex = new int[TriggerTreeStackCapacity];
            int depth = 0;
            bool contact = false;

            CPlugTree root = carTrees[carTree];
            GmIso4 carWorld = CollisionHelpers.TreeWorldIso(root, ref carWorldTransform);
            if (root.surface != null)
            {
                contact |= Narrowphase.CplugSurfaceComputeCollision(
                    root.surface, ref carWorld, mesh, ref triggerIso, buffer, runtime) != 0;
            }
            if (root.children.Length > TriggerTreeStackCapacity)
            {
                throw new InvalidOperationException("tmnf race: car collision tree is wider than the trigger stack");
            }
            int i2 = root.children.Length;
            while (i2 > 0)
            {
                i2 -= 1;
                stackIndex[depth] = root.children[i2];
                stackIso[depth] = carWorld;
                depth += 1;
            }
            while (depth != 0)
            {
                depth -= 1;
                int frameIndex = stackIndex[depth];
                GmIso4 frameIso = stackIso[depth];
                CPlugTree tree = carTrees[frameIndex];
                if ((tree.flags & 0x80u) == 0)
                {
                    continue;
                }
                GmBoxAligned boxAligned = default(GmBoxAligned);
                boxAligned.SetMult(ref tree.boxAligned, ref frameIso);
                if (!boxAligned.TestInter(ref triggerWorldBox))
                {
                    continue;
                }
                GmIso4 world = CollisionHelpers.TreeWorldIso(tree, ref frameIso);
                if (tree.surface != null)
                {
                    contact |= Narrowphase.CplugSurfaceComputeCollision(
                        tree.surface, ref world, mesh, ref triggerIso, buffer, runtime) != 0;
                }
                if (depth + tree.children.Length > TriggerTreeStackCapacity)
                {
                    throw new InvalidOperationException("tmnf race: car collision tree is deeper than the trigger stack");
                }
                int i3 = tree.children.Length;
                while (i3 > 0)
                {
                    i3 -= 1;
                    stackIndex[depth] = tree.children[i3];
                    stackIso[depth] = world;
                    depth += 1;
                }
            }
            return contact;
        }

        /// <summary>TmnfRace_TriggerContact: game-faithful trigger contact.</summary>
        public static bool TmnfRaceTriggerContact(
            RouteLoader.TmnfRouteTrigger trigger,
            CPlugTree[] carTrees,
            int carTree,
            ref GmIso4 carWorldTransform)
        {
            CPlugTree root = carTrees[carTree];
            if ((root.flags & 0x80u) == 0)
            {
                throw new InvalidOperationException("tmnf race: car collision tree has collisions disabled");
            }
            if ((trigger.treeFlags & 0x80u) == 0)
            {
                throw new InvalidOperationException("tmnf race: trigger tree has collisions disabled");
            }

            GmBoxAligned carBox = default(GmBoxAligned);
            carBox.SetMult(ref root.boxAligned, ref carWorldTransform);
            GmBoxAligned triggerBox = default(GmBoxAligned);
            GmBoxAligned triggerLocalBox = trigger.boxAligned;
            GmIso4 triggerTransform = trigger.transform;
            triggerBox.SetMult(ref triggerLocalBox, ref triggerTransform);
            if (!carBox.TestInter(ref triggerBox))
            {
                return false;
            }

            CPlugSurface mesh = BuildTriggerMesh(ref triggerLocalBox);
            Narrowphase.CollisionShapeDispatch runtime = Narrowphase.CollisionShapeDispatch.Default();
            // The contacts themselves are discarded; only their existence counts.
            CHmsCollisionBuffer buffer = new CHmsCollisionBuffer();
            bool contact = CarTreeContact(
                carTrees, carTree, ref carWorldTransform, ref triggerBox,
                mesh, ref triggerTransform, buffer, runtime);
            return contact;
        }

        /// <summary>TmnfRace_TriggerContactMask: every trigger vs the car tree.</summary>
        public static ulong TmnfRaceTriggerContactMask(
            RouteLoader.TmnfRoute route,
            CPlugTree[] carTrees,
            int carTree,
            ref GmIso4 carWorldTransform)
        {
            ValidateRoute(route);
            ulong mask = 0;
            for (uint i = 0; i < route.metadata.checkpointCount; i++)
            {
                if (TmnfRaceTriggerContact(
                    route.checkpoints[(int)i], carTrees, carTree, ref carWorldTransform))
                {
                    mask |= 1UL << (int)i;
                }
            }
            for (uint i = 0; i < route.metadata.finishCount; i++)
            {
                if (TmnfRaceTriggerContact(
                    route.finish[(int)i], carTrees, carTree, ref carWorldTransform))
                {
                    mask |= 1UL << (int)(TmnfRaceFinishContactBit - i);
                }
            }
            return mask;
        }

        // ------------------------------------------------------------------
        // dense projection
        // ------------------------------------------------------------------

        private struct DenseProjection
        {
            public RouteLoader.TmnfRouteProjection projection;
            public float lateralHorizontal;
            public float vertical;
        }

        /// <summary>project_segment: (projection, distance_sq, offset).</summary>
        private static (RouteLoader.TmnfRouteProjection, float, GmVec3) ProjectSegment(
            RouteLoader.TmnfRoute route,
            ref GmVec3 position,
            uint segment)
        {
            RouteLoader.TmnfRouteReferencePoint[] points = route.centerline.ToArray();
            RouteLoader.TmnfRouteReferencePoint a = points[segment];
            RouteLoader.TmnfRouteReferencePoint b = points[segment + 1];
            float dx = (float)(double)(b.position.x - a.position.x);
            float dy = (float)(double)(b.position.y - a.position.y);
            float dz = (float)(double)(b.position.z - a.position.z);
            float px = (float)(double)(position.x - a.position.x);
            float py = (float)(double)(position.y - a.position.y);
            float pz = (float)(double)(position.z - a.position.z);
            float lengthSq = (float)(double)((float)(double)(((float)(double)(dx * dx)) + ((float)(double)(dy * dy))) + ((float)(double)(dz * dz)));
            if (!(lengthSq > 0.0f))
            {
                throw new InvalidOperationException("tmnf race: route centerline contains a degenerate segment");
            }
            float t = (float)(double)(((float)(double)((float)(double)((float)(double)(px * dx) + (float)(double)(py * dy)) + (float)(double)(pz * dz))) / lengthSq);
            if (t < 0.0f)
            {
                t = 0.0f;
            }
            else if (t > 1.0f)
            {
                t = 1.0f;
            }
            float ox = (float)(double)(px - (float)(double)(t * dx));
            float oy = (float)(double)(py - (float)(double)(t * dy));
            float oz = (float)(double)(pz - (float)(double)(t * dz));
            float distanceSq = (float)(double)((float)(double)(((float)(double)(ox * ox)) + ((float)(double)(oy * oy))) + ((float)(double)(oz * oz)));
            GmVec3 offset = new GmVec3(ox, oy, oz);
            RouteLoader.TmnfRouteProjection projection = default(RouteLoader.TmnfRouteProjection);
            projection.arcLength = (float)(double)(a.arcLength + (float)(double)(t * ((float)(double)(b.arcLength - a.arcLength))));
            projection.lateralOffset = 0.0f;
            projection.halfWidth = (float)(double)(a.halfWidth + (float)(double)(t * ((float)(double)(b.halfWidth - a.halfWidth))));
            projection.segmentIndex = a.legIndex;
            projection.centerlineSegmentIndex = segment;
            projection.segmentsTested = 1;
            return (projection, distanceSq, offset);
        }

        private static DenseProjection DenseFromOffset(
            RouteLoader.TmnfRouteProjection projection, ref GmVec3 offset)
        {
            DenseProjection d = default(DenseProjection);
            d.projection = projection;
            d.lateralHorizontal = Fp.Sqrt((float)(double)(((float)(double)(offset.x * offset.x)) + ((float)(double)(offset.z * offset.z))));
            d.vertical = offset.y;
            return d;
        }

        /// <summary>project_full: full-route search (reset, teleport).</summary>
        private static DenseProjection ProjectFull(RouteLoader.TmnfRoute route, ref GmVec3 position)
        {
            RouteLoader.TmnfRouteProjection projection = RouteQueries.Project(route, ref position);
            var (_, _, offset) = ProjectSegment(route, ref position, projection.centerlineSegmentIndex);
            return DenseFromOffset(projection, ref offset);
        }

        /// <summary>project_local: the 65-segment dense window around the cursor.</summary>
        private static DenseProjection ProjectLocal(RouteLoader.TmnfRoute route, ref TmnfRaceState state, ref GmVec3 position)
        {
            uint segmentCount = (uint)route.centerline.Count - 1;
            if (state.centerlineSegment >= segmentCount)
            {
                throw new InvalidOperationException("tmnf race: dense projection cursor is out of range");
            }
            uint minimum = state.centerlineSegment > TmnfRaceProjectionWindowSegments
                ? state.centerlineSegment - TmnfRaceProjectionWindowSegments
                : 0;
            uint maximum = unchecked(state.centerlineSegment + TmnfRaceProjectionWindowSegments);
            if (maximum >= segmentCount)
            {
                maximum = segmentCount - 1;
            }

            var (best, bestDistanceSq, bestOffset) = ProjectSegment(route, ref position, minimum);
            for (uint segment = minimum + 1; segment <= maximum; segment++)
            {
                var (candidate, candidateDistanceSq, candidateOffset) =
                    ProjectSegment(route, ref position, segment);
                if (candidateDistanceSq < bestDistanceSq)
                {
                    best = candidate;
                    bestDistanceSq = candidateDistanceSq;
                    bestOffset = candidateOffset;
                }
            }
            best.lateralOffset = Fp.Sqrt(bestDistanceSq);
            best.segmentsTested = maximum - minimum + 1;
            return DenseFromOffset(best, ref bestOffset);
        }

        /// <summary>project_dense: window or full reacquisition on teleport.</summary>
        private static DenseProjection ProjectDense(RouteLoader.TmnfRoute route, ref TmnfRaceState state, ref GmVec3 position)
        {
            if (!float.IsFinite(position.x) || !float.IsFinite(position.y) || !float.IsFinite(position.z))
            {
                throw new InvalidOperationException("tmnf race: cannot project a non-finite position");
            }
            float dx = (float)(double)(position.x - state.previousCarTransform.t0);
            float dy = (float)(double)(position.y - state.previousCarTransform.t1);
            float dz = (float)(double)(position.z - state.previousCarTransform.t2);
            float displacementSq = (float)(double)((float)(double)(((float)(double)(dx * dx)) + ((float)(double)(dy * dy))) + ((float)(double)(dz * dz)));
            float teleportDistance = TmnfRaceTeleportDistanceMeters;
            if (displacementSq > (float)(double)(teleportDistance * teleportDistance))
            {
                return ProjectFull(route, ref position);
            }
            return ProjectLocal(route, ref state, ref position);
        }

        /// <summary>apply_projection.</summary>
        private static void ApplyProjection(ref TmnfRaceState state, ref DenseProjection dense)
        {
            state.arcLength = dense.projection.arcLength;
            state.lateralOffset = dense.projection.lateralOffset;
            state.halfWidth = dense.projection.halfWidth;
            state.projectionSegment = dense.projection.segmentIndex;
            state.centerlineSegment = dense.projection.centerlineSegmentIndex;
            state.corridorLateral = dense.lateralHorizontal;
            state.corridorVertical = dense.vertical;
            float allowed = (float)(double)(TmnfRaceCorridorWidthFactor * dense.projection.halfWidth);
            if (allowed < TmnfRaceCorridorWidthFloorMeters)
            {
                allowed = TmnfRaceCorridorWidthFloorMeters;
            }
            state.outsideCorridor = (byte)((dense.lateralHorizontal > allowed
                || dense.vertical > TmnfRaceCorridorAboveMeters
                || dense.vertical < -TmnfRaceCorridorBelowMeters) ? 1 : 0);
        }

        // ------------------------------------------------------------------
        // public race API
        // ------------------------------------------------------------------

        /// <summary>TmnfRace_IsGroundPlaneMaterial.</summary>
        public static bool TmnfRaceIsGroundPlaneMaterial(int materialId)
        {
            return materialId == (int)TmnfMaterialGrass || materialId == (int)TmnfMaterialWetGrass;
        }

        /// <summary>TmnfRace_Reset.</summary>
        public static void TmnfRaceReset(RouteLoader.TmnfRoute route, ref TmnfRaceState state, ref GmIso4 carWorldTransform)
        {
            ValidateRoute(route);
            // Rust: *state = TmnfRaceState::default() — memset(state, 0, ...):
            // an all-zero struct (both isos zero matrices; the two lines
            // below immediately overwrite them).
            state = default(TmnfRaceState);
            state.previousCarTransform = carWorldTransform;
            state.respawnLocation = route.start.spawn;
            GmVec3 position = new GmVec3(
                carWorldTransform.t0,
                carWorldTransform.t1,
                carWorldTransform.t2);
            DenseProjection dense = ProjectFull(route, ref position);
            ApplyProjection(ref state, ref dense);
            state.unwrappedProgress = dense.projection.arcLength;
            state.bestProgress = dense.projection.arcLength;
            state.previousProgress = dense.projection.arcLength;
        }

        /// <summary>TmnfRace_UpdateOffTrack.</summary>
        public static bool TmnfRaceUpdateOffTrack(
            ref TmnfRaceState state,
            uint graceTicks,
            uint wheelsInContact,
            uint wheelsOnGroundPlane)
        {
            if (graceTicks == 0 || wheelsOnGroundPlane > wheelsInContact)
            {
                throw new InvalidOperationException("tmnf race: invalid off-track state");
            }
            if (wheelsInContact == 0)
            {
                return false;
            }
            if (wheelsOnGroundPlane != wheelsInContact)
            {
                state.offTrackTicks = 0;
                return false;
            }
            if (state.offTrackTicks < graceTicks)
            {
                state.offTrackTicks += 1;
            }
            return state.offTrackTicks == graceTicks;
        }

        /// <summary>TmnfRace_Step: one canonical 10 ms tick of race logic.</summary>
        public static TmnfRaceStepResult TmnfRaceStep(
            RouteLoader.TmnfRoute route,
            ref TmnfRaceState state,
            ulong triggerContacts,
            ref GmIso4 carWorldTransform)
        {
            ValidateRoute(route);
            if (state.finished != 0)
            {
                throw new InvalidOperationException("tmnf race: cannot step a finished race");
            }

            TmnfRaceStepResult result = default(TmnfRaceStepResult);
            uint checkpointCount = route.metadata.checkpointCount;
            state.elapsedTicks = unchecked(state.elapsedTicks + 1);

            ulong entered = triggerContacts & ~state.triggerContacts;
            state.triggerContacts = triggerContacts;

            for (uint i = 0; i < checkpointCount; i++)
            {
                ulong bit = 1UL << (int)i;
                if ((entered & bit) == 0)
                {
                    continue;
                }
                if ((state.visitedCheckpoints & bit) != 0)
                {
                    result.checkpointRepeated = 1;
                    continue;
                }
                state.visitedCheckpoints |= bit;
                state.visitedCount += 1;
                state.passedRaceCheckpoints += 1;
                result.checkpointAccepted = 1;
                result.checkpointIndex = i;
                if (route.checkpoints[(int)i].noRespawn == 0)
                {
                    state.respawnLocation = route.checkpoints[(int)i].spawn;
                    state.respawnAvailable = 1;
                }
            }

            bool enteredFinish = (entered & (ulong.MaxValue << (int)(64 - route.metadata.finishCount))) != 0;
            bool beganNewLap = false;
            if (enteredFinish && state.visitedCount == checkpointCount)
            {
                state.passedRaceCheckpoints += 1;
                state.completedLaps += 1;
                result.lapCompleted = 1;
                if (state.completedLaps == route.metadata.lapCount)
                {
                    if (state.passedRaceCheckpoints != route.metadata.totalRaceCheckpoints)
                    {
                        throw new InvalidOperationException("tmnf race: finished with the wrong checkpoint total");
                    }
                    state.finished = 1;
                    state.finishTimeMs = unchecked(state.elapsedTicks * TmnfRaceTickMs);
                    result.finished = 1;
                    result.raceTimeMs = state.finishTimeMs;
                }
                else
                {
                    state.visitedCheckpoints = 0;
                    state.visitedCount = 0;
                    state.respawnLocation = route.start.spawn;
                    state.projectionSegment = route.centerline[0].legIndex;
                    state.centerlineSegment = 0;
                    state.arcLength = 0.0f;
                    state.lateralOffset = 0.0f;
                    state.halfWidth = route.centerline[0].halfWidth;
                    state.corridorLateral = 0.0f;
                    state.corridorVertical = 0.0f;
                    state.outsideCorridor = 0;
                    beganNewLap = true;
                }
            }

            if (state.finished != 0)
            {
                uint pointCount = (uint)route.centerline.Count;
                state.arcLength = route.centerline[route.centerline.Count - 1].arcLength;
                state.lateralOffset = 0.0f;
                state.halfWidth = route.centerline[(int)pointCount - 1].halfWidth;
                state.centerlineSegment = pointCount - 2;
                state.corridorLateral = 0.0f;
                state.corridorVertical = 0.0f;
                state.outsideCorridor = 0;
            }
            else if (!beganNewLap)
            {
                GmVec3 position = new GmVec3(
                    carWorldTransform.t0,
                    carWorldTransform.t1,
                    carWorldTransform.t2);
                DenseProjection dense = ProjectDense(route, ref state, ref position);
                ApplyProjection(ref state, ref dense);
            }

            state.previousProgress = state.unwrappedProgress;
            if (state.finished != 0 || beganNewLap)
            {
                state.unwrappedProgress =
                    (float)(double)(state.completedLaps * route.centerline[route.centerline.Count - 1].arcLength);
            }
            else if (state.outsideCorridor == 0)
            {
                state.unwrappedProgress = (float)(double)((float)(double)(state.completedLaps
                    * route.centerline[route.centerline.Count - 1].arcLength)
                    + state.arcLength);
            }
            if (state.unwrappedProgress > state.bestProgress)
            {
                state.bestProgress = state.unwrappedProgress;
            }
            state.previousCarTransform = carWorldTransform;
            return result;
        }

        /// <summary>TmnfRace_RespawnLocation (null = none).</summary>
        public static GmIso4? TmnfRaceRespawnLocation(ref TmnfRaceState state)
        {
            if (state.finished != 0)
            {
                throw new InvalidOperationException("tmnf race: cannot respawn a finished race");
            }
            if (state.respawnAvailable != 0)
            {
                return state.respawnLocation;
            }
            return null;
        }
    }
}
