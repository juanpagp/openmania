using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// The collision narrowphase: sphere/sphere, sphere/mesh, ellipsoid/mesh,
    /// box/mesh kernels, the sphere-buffer merge and the static grid locate —
    /// transliterated with the game's exact operand orders, including the
    /// double-square-root end-vertex quirk.
    /// </summary>
    public static class Narrowphase
    {
        // 0x1.b7cdfcp-34f — the edge-length/vertex-distance floor
        private static readonly float TinySquared =
            BitConverter.Int32BitsToSingle(unchecked((int)0x2EB7_CDFC));
        // 0x1.4f8b58p-17f — the edge closest-point squared floor
        private static readonly float EdgeClosestSquared =
            BitConverter.Int32BitsToSingle(unchecked((int)0x374F_8B58));
        // 0x1.4f8b588e368f1p-17 (f64) — GmVec3_IsNearlyEqual scale
        private static readonly double NearlyEqualScale =
            BitConverter.Int64BitsToDouble(unchecked((long)0x3EE4_F8B5_88E3_68F1));
        // cos(pi/6) 0x3f5db3d7 — the sphere merge threshold
        private static readonly float SphereMergeThreshold =
            BitConverter.Int32BitsToSingle(unchecked((int)0x3F5D_B3D7));

        private static float CollisionDot3(ref GmVec3 a, ref GmVec3 b)
        {
            return (float)(double)(((float)(double)(((float)(double)(a.x * b.x)) + ((float)(double)(a.y * b.y)))) + ((float)(double)(a.z * b.z)));
        }

        private static float CollisionLengthSquared(ref GmVec3 value)
        {
            return CollisionDot3(ref value, ref value);
        }

        // ------------------------------------------------------------------
        // mesh list selection (grid accel)
        // ------------------------------------------------------------------

        /// <summary>
        /// Selects the region-restricted node list of the finest grid level
        /// whose budget covers the query box. Returns null for the full array.
        /// NaN coordinates fail every comparison and take the full array.
        /// </summary>
        internal static MeshListNodeList? SelectMeshList(
            GmSurfMesh mesh,
            TmnfMeshQueryAccel accel,
            ref GmBoxAligned meshBox)
        {
            uint fullCount = mesh.nodeCount;
            if (accel == null)
            {
                return null;
            }
            TmnfMeshGrid grid = accel.meshGrid;
            if (grid == null)
            {
                return null;
            }
            float[] c = { meshBox.center.x, meshBox.center.y, meshBox.center.z };
            float[] h = { meshBox.halfExtent.x, meshBox.halfExtent.y, meshBox.halfExtent.z };
            foreach (TmnfMeshGridLevel level in grid.levels)
            {
                uint cell = 0;
                int a = 2;
                while (a >= 0)
                {
                    if (!((double)h[a] <= level.maxHalf))
                    {
                        break;
                    }
                    double offset = (double)c[a] - level.origin[a];
                    if (!(offset >= 0.0 && offset < (double)level.dims[a] * level.cellSize))
                    {
                        break;
                    }
                    uint index = (uint)(offset * level.invCellSize);
                    if (index >= level.dims[a])
                    {
                        break;
                    }
                    cell = cell * level.dims[a] + index;
                    a -= 1;
                }
                if (a >= 0)
                {
                    continue;
                }
                uint header = level.cells[(int)cell];
                uint count = grid.pool[(int)header].NodeCount();
                MeshListNodeList list = new MeshListNodeList
                {
                    start = header + 1,
                    count = count,
                };
                return list;
            }
            return null;
        }

        // ------------------------------------------------------------------
        // sphere buffer merge
        // ------------------------------------------------------------------

        private static bool Gmvec3IsNearlyEqual(ref GmVec3 self, ref GmVec3 other)
        {
            float absolute = Math.Abs(other.x);
            float tolerance = (float)(NearlyEqualScale * (double)absolute);
            float lower = (float)(double)(other.x - tolerance);
            float upper = (float)(double)(other.x + tolerance);
            if (!(lower <= self.x) || !(self.x <= upper))
            {
                return false;
            }

            absolute = Math.Abs(other.y);
            tolerance = (float)(NearlyEqualScale * (double)absolute);
            lower = (float)(double)(other.y - tolerance);
            upper = (float)(double)(other.y + tolerance);
            if (!(lower <= self.y) || !(self.y <= upper))
            {
                return false;
            }

            absolute = Math.Abs(other.z);
            tolerance = (float)(NearlyEqualScale * (double)absolute);
            lower = (float)(double)(other.z - tolerance);
            upper = (float)(double)(other.z + tolerance);
            return lower <= self.z && self.z <= upper;
        }

        /// <summary>0x00538100: merges a per-sphere buffer into the destination.</summary>
        public static void ShmsSphereBufferContactMergeAndAddToCollisions(
            SHmsSphereBufferContact @this,
            CHmsCollisionBuffer destination)
        {
            int sourceCount = (int)@this.@base.collisions.Count();
            int destinationStart = (int)destination.collisions.Count();

            for (int i = 0; i < sourceCount; i++)
            {
                SHmsPhysicalCollision source = @this.@base.collisions.At(i);
                if (source.collision.flags != 0)
                {
                    destination.collisions.Push(source);
                }
            }

            int destinationEnd = (int)destination.collisions.Count();
            for (int i = 0; i < sourceCount; i++)
            {
                SHmsPhysicalCollision source = @this.@base.collisions.At(i);
                if (source.collision.flags != 0)
                {
                    continue;
                }

                int j = destinationStart;
                while (j < destinationEnd)
                {
                    SHmsPhysicalCollision existing = destination.collisions.At(j);
                    if (Gmvec3IsNearlyEqual(ref source.collision.faceNormal, ref existing.collision.faceNormal))
                    {
                        break;
                    }
                    float dot = ((float)(double)(((float)(double)(((float)(double)(source.collision.normal.z * existing.collision.normal.z))
                        + ((float)(double)(source.collision.normal.x * existing.collision.normal.x))))
                        + ((float)(double)(source.collision.normal.y * existing.collision.normal.y))));
                    if (SphereMergeThreshold < dot)
                    {
                        break;
                    }
                    j += 1;
                }
                if (j == destinationEnd)
                {
                    destination.collisions.Push(source);
                }
            }

            @this.active = 0;
            @this.@base.collisions.Clear();
        }

        // ------------------------------------------------------------------
        // sphere / sphere
        // ------------------------------------------------------------------

        /// <summary>0x008F49D0: sphere/sphere overlap and contact data.</summary>
        public static int GmCollisionSphereSphere(
            ref LocatedGmSurf sphere1,
            ref LocatedGmSurf sphere2,
            CHmsCollisionBuffer buffer)
        {
            GmSurfSphere shape1 = sphere1.surf.AsSphere();
            GmSurfSphere shape2 = sphere2.surf.AsSphere();

            float dx = (float)(double)(sphere2.iso.t0 - sphere1.iso.t0);
            float dy = (float)(double)(sphere2.iso.t1 - sphere1.iso.t1);
            float dz = (float)(double)(sphere2.iso.t2 - sphere1.iso.t2);
            float distanceSquared = (float)(double)(((float)(double)(((float)(double)(dx * dx)) + ((float)(double)(dy * dy)))) + ((float)(double)(dz * dz)));
            // Load-bearing f64: the radius sum and its square are computed in
            // double and compared in double (never stored to float).
            double radiusSumX87 = (double)shape2.radius + (double)shape1.radius;
            double radiusSquared = radiusSumX87 * radiusSumX87;
            if (!((double)distanceSquared < radiusSquared))
            {
                return 0;
            }

            float distance = Fp.Sqrt(distanceSquared);
            if (distance <= 1.0e-5f)
            {
                ref GmCollision c = ref buffer.AddCollision();
                c.normal.x = 0.0f;
                c.normal.y = -1.0f;
                c.normal.z = 0.0f;
                c.separation.x = 0.0f;
                c.separation.y = shape2.radius;
                c.separation.z = 0.0f;
                c.position.x = sphere1.iso.t0;
                c.position.y = sphere1.iso.t1;
                c.position.z = sphere1.iso.t2;
            }
            else
            {
                float inverseDistance = (float)(double)(1.0f / distance);
                dx = (float)(double)(inverseDistance * dx);
                dy = (float)(double)(dy * inverseDistance);
                dz = (float)(double)(inverseDistance * dz);

                ref GmCollision c = ref buffer.AddCollision();
                c.normal.x = -dx;
                c.normal.y = -dy;
                c.normal.z = -dz;

                float penetration = (float)(double)(((float)(double)(shape2.radius + shape1.radius)) - distance);
                c.separation.x = (float)(double)(penetration * dx);
                c.separation.y = (float)(double)(dy * penetration);
                c.separation.z = (float)(double)(penetration * dz);

                float radius1 = shape1.radius;
                c.position.x = (float)(double)(radius1 * dx);
                c.position.y = (float)(double)(dy * radius1);
                c.position.z = (float)(double)(radius1 * dz);
                c.position.x = (float)(double)(sphere1.iso.t0 + c.position.x);
                c.position.y = (float)(double)(sphere1.iso.t1 + c.position.y);
                c.position.z = (float)(double)(sphere1.iso.t2 + c.position.z);
            }
            ref SHmsPhysicalCollision last = ref buffer.collisions.At(buffer.collisions.count - 1);
            last.collision.material1 = shape1.@base.materialIndex;
            last.collision.material2 = shape2.@base.materialIndex;
            return 1;
        }

        // ------------------------------------------------------------------
        // sphere / mesh
        // ------------------------------------------------------------------

        private static void FinishMeshCollision(
            ref GmCollision collision,
            ref GmSurf first,
            ref GmSurfMeshFace face,
            ref GmVec3 position,
            ref GmVec3 normal,
            ref GmVec3 separation,
            uint flags)
        {
            collision.normal = normal;
            collision.separation = separation;
            collision.position = position;
            collision.material1 = first.materialIndex;
            collision.material2 = face.materialIndex;
            collision.flags = flags;
            collision.faceNormal = face.normal;
        }

        /// <summary>
        /// Sphere feature contact. dx/dy/dz is center - feature and distance
        /// is the value the game uses as |center - feature| for this branch;
        /// the end-vertex branch passes sqrt(sqrt(feature_squared)) — the
        /// game square-roots twice (kept on purpose).
        /// </summary>
        private static void AddSphereFeatureCollision(
            ref GmSurfSphere sphere,
            ref GmSurfMeshFace face,
            ref GmVec3 position,
            float dx, float dy, float dz, float distance,
            CHmsCollisionBuffer buffer)
        {
            float inverseDistance = (float)(double)(1.0f / distance);
            GmVec3 normal = new GmVec3(
                (float)(double)(inverseDistance * dx),
                (float)(double)(dy * inverseDistance),
                (float)(double)(inverseDistance * dz));
            float scale = (float)(double)(((float)(double)(distance - sphere.radius)) * inverseDistance);
            GmVec3 radialSeparation = new GmVec3(
                (float)(double)(scale * dx),
                (float)(double)(dy * scale),
                (float)(double)(scale * dz));
            float planeSeparation = (float)(double)(((float)(double)(((float)(double)(face.normal.x * radialSeparation.x))
                + ((float)(double)(radialSeparation.y * face.normal.y))))
                + ((float)(double)(radialSeparation.z * face.normal.z)));
            GmVec3 separation = new GmVec3(
                (float)(double)(planeSeparation * face.normal.x),
                (float)(double)(planeSeparation * face.normal.y),
                (float)(double)(planeSeparation * face.normal.z));
            ref GmCollision collision = ref buffer.AddCollision();
            FinishMeshCollision(ref collision, ref sphere.@base, ref face,
                ref position, ref normal, ref separation, 0);
        }

        /// <summary>
        /// 0x008EA2D0: sphere contacts against a triangle mesh. The three
        /// feature branches differ in the game and are reproduced as-is
        /// (start vertex: reject when feature_squared &gt; r*r or &lt;= 1e-10;
        /// end vertex: double square root; edge: 1e-5 floor, no radius test).
        /// </summary>
        public static int GmCollisionSphereMesh(
            ref LocatedGmSurf sphereLocated,
            ref LocatedGmSurf meshLocated,
            CHmsCollisionBuffer buffer)
        {
            // 0x00D1A938: 1e-10, the edge-length and vertex-distance floor
            float sphereTinySquared = TinySquared;
            GmSurfSphere sphere = sphereLocated.surf.AsSphere();
            GmSurfMesh mesh = meshLocated.surf.AsMesh();
            TmnfSphereFaceEdges[] cachedEdges = null;
            if (meshLocated.accel != null && meshLocated.accel.meshGrid != null)
            {
                cachedEdges = meshLocated.accel.meshGrid.sphereEdges;
            }
            GmIso4 relative = CollisionHelpers.RelativeIso(ref sphereLocated, ref meshLocated);

            uint start = buffer.GetCount();
            GmBoxAligned sphereBox = default(GmBoxAligned);
            sphereBox.center = GmVec3.Zero;
            sphereBox.halfExtent = new GmVec3(sphere.radius, sphere.radius, sphere.radius);
            GmBoxAligned transformedBox = default(GmBoxAligned);
            transformedBox.SetMult(ref sphereBox, ref relative);
            GmVec3 center = transformedBox.center;
            int result = 0;

            MeshListNodeList? list = SelectMeshList(mesh, meshLocated.accel, ref transformedBox);
            TmnfMeshPoolSlot[] listPool = null;
            if (meshLocated.accel != null && meshLocated.accel.meshGrid != null)
            {
                listPool = meshLocated.accel.meshGrid.pool;
            }
            uint nodeIndex = 0;
            uint nodeCount = list?.count ?? mesh.nodeCount;
            while (nodeIndex < nodeCount)
            {
                int nodeRef;
                uint skipCount;
                if (list.HasValue && listPool != null)
                {
                    TmnfMeshPoolSlot slot = listPool[(int)(list.Value.start + nodeIndex)];
                    nodeRef = (int)slot.EntryNode();
                    skipCount = slot.EntrySkip();
                }
                else
                {
                    GmSurfMeshNode n = mesh.nodes[nodeIndex];
                    nodeRef = (int)nodeIndex;
                    skipCount = n.skipCount;
                }
                GmSurfMeshNode node = mesh.nodes[nodeRef];
                if (!transformedBox.TestInter(ref node.boxAligned))
                {
                    nodeIndex += skipCount;
                    continue;
                }
                if (node.faceIndex == uint.MaxValue)
                {
                    nodeIndex += 1;
                    continue;
                }

                GmSurfMeshFace face = mesh.faces[node.faceIndex];
                GmVec3[] vertices =
                {
                    mesh.vertices[face.vertex0],
                    mesh.vertices[face.vertex1],
                    mesh.vertices[face.vertex2],
                };
                float distance = (float)(double)(((float)(double)(((float)(double)(((float)(double)(center.y - vertices[0].y)) * face.normal.y))
                    + ((float)(double)(face.normal.x * ((float)(double)(center.x - vertices[0].x))))))
                    + ((float)(double)(((float)(double)(center.z - vertices[0].z)) * face.normal.z)));
                if (!(distance <= sphere.radius) || !(0.0f <= distance))
                {
                    nodeIndex += 1;
                    continue;
                }

                float sectionSquared = (float)(double)(((float)(double)(sphere.radius * sphere.radius)) - ((float)(double)(distance * distance)));
                float sectionRadius = Fp.Sqrt(sectionSquared);
                float negativeDistance = -distance;
                GmVec3 projected = new GmVec3(
                    (float)(double)(((float)(double)(face.normal.x * negativeDistance)) + center.x),
                    (float)(double)(((float)(double)(face.normal.y * negativeDistance)) + center.y),
                    (float)(double)(((float)(double)(negativeDistance * face.normal.z)) + center.z));

                int emitted = 0;
                for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
                {
                    int nextIndex = edgeIndex == 2 ? 0 : edgeIndex + 1;
                    GmVec3 vertex = vertices[edgeIndex];
                    GmVec3 next = vertices[nextIndex];
                    TmnfSphereMeshEdge prepared = default(TmnfSphereMeshEdge);
                    bool havePrepared = false;
                    if (cachedEdges != null)
                    {
                        prepared = cachedEdges[node.faceIndex].edges[edgeIndex];
                        havePrepared = true;
                    }
                    if (!havePrepared)
                    {
                        prepared = CollisionHelpers.TmnfSphereMeshEdgePrepare(ref vertex, ref next, ref face.normal);
                    }
                    GmVec3 edge = prepared.direction;
                    GmVec3 side = prepared.side;
                    GmVec3 fromVertex = new GmVec3(
                        (float)(double)(projected.x - vertex.x),
                        (float)(double)(projected.y - vertex.y),
                        (float)(double)(projected.z - vertex.z));
                    float sideDistance = (float)(double)(((float)(double)(((float)(double)(fromVertex.x * side.x)) + ((float)(double)(fromVertex.y * side.y))))
                        + ((float)(double)(fromVertex.z * side.z)));
                    if (sectionRadius < sideDistance)
                    {
                        emitted = -1;
                        break;
                    }
                    if (!(0.0f < sideDistance))
                    {
                        continue;
                    }

                    float alongStart = (float)(double)(((float)(double)(((float)(double)(fromVertex.x * edge.x)) + ((float)(double)(fromVertex.y * edge.y))))
                        + ((float)(double)(fromVertex.z * edge.z)));
                    GmVec3 closest = GmVec3.Zero;
                    bool isEdge = false;
                    bool isEndVertex = false;
                    GmVec3 feature;
                    if (0.0f <= alongStart)
                    {
                        GmVec3 fromNext = new GmVec3(
                            (float)(double)(projected.x - next.x),
                            (float)(double)(projected.y - next.y),
                            (float)(double)(projected.z - next.z));
                        float alongEnd = (float)(double)(((float)(double)(((float)(double)(fromNext.x * edge.x)) + ((float)(double)(fromNext.y * edge.y))))
                            + ((float)(double)(fromNext.z * edge.z)));
                        if (alongEnd <= 0.0f)
                        {
                            float mv = -sideDistance;
                            closest.x = (float)(double)(((float)(double)(mv * side.x)) + projected.x);
                            closest.y = (float)(double)(projected.y + ((float)(double)(side.y * mv)));
                            closest.z = (float)(double)(((float)(double)(mv * side.z)) + projected.z);
                            feature = closest;
                            isEdge = true;
                        }
                        else
                        {
                            feature = next;
                            isEndVertex = true;
                        }
                    }
                    else
                    {
                        feature = vertex;
                    }

                    float dx = (float)(double)(center.x - feature.x);
                    float dy = (float)(double)(center.y - feature.y);
                    float dz = (float)(double)(center.z - feature.z);
                    float featureSquared = (float)(double)(((float)(double)(((float)(double)(dx * dx)) + ((float)(double)(dy * dy)))) + ((float)(double)(dz * dz)));
                    float featureDistance;
                    if (isEdge)
                    {
                        if (featureSquared <= 1.0e-5f)
                        {
                            emitted = -1;
                            break;
                        }
                        featureDistance = Fp.Sqrt(featureSquared);
                    }
                    else
                    {
                        float radiusSquared = (float)(double)(sphere.radius * sphere.radius);
                        float tested = isEndVertex ? Fp.Sqrt(featureSquared) : featureSquared;
                        if (!(tested <= radiusSquared) || tested <= sphereTinySquared)
                        {
                            emitted = -1;
                            break;
                        }
                        featureDistance = Fp.Sqrt(tested);
                    }
                    AddSphereFeatureCollision(
                        ref sphere, ref face, ref feature, dx, dy, dz, featureDistance, buffer);
                    result = 1;
                    emitted = 1;
                    break;
                }

                if (emitted == 0 && 0.0f < distance)
                {
                    float depth = (float)(double)(distance - sphere.radius);
                    GmVec3 separation = new GmVec3(
                        (float)(double)(depth * face.normal.x),
                        (float)(double)(depth * face.normal.y),
                        (float)(double)(depth * face.normal.z));
                    ref GmCollision collision = ref buffer.AddCollision();
                    FinishMeshCollision(ref collision, ref sphere.@base, ref face,
                        ref projected, ref face.normal, ref separation, 1);
                    result = 1;
                }
                nodeIndex += 1;
            }

            if (result != 0)
            {
                uint end = buffer.GetCount();
                for (uint i = start; i < end; i++)
                {
                    // The record is mutated in place inside the buffer.
                    ref GmCollision c = ref buffer.GetCollision((int)i);
                    GmIso4 meshIso = meshLocated.iso;
                    CollisionHelpers.TransformDirection(ref c.normal, ref meshIso);
                    CollisionHelpers.TransformDirection(ref c.separation, ref meshIso);
                    c.position.MultIso4(ref meshIso);
                }
            }
            return result;
        }

        // ------------------------------------------------------------------
        // ellipsoid / mesh
        // ------------------------------------------------------------------

        private static (GmIso4, GmIso4) MakeEllipsoidInverse(ref GmIso4 relative, ref GmVec3 radii)
        {
            GmIso4 inverse = default(GmIso4);
            inverse.SetInverse(ref relative);
            GmIso4 scaledInverse = inverse;

            float inverseX = (float)(double)(1.0f / radii.x);
            scaledInverse.m0 = (float)(double)(inverseX * scaledInverse.m0);
            scaledInverse.m1 = (float)(double)(scaledInverse.m1 * inverseX);
            scaledInverse.m2 = (float)(double)(inverseX * scaledInverse.m2);
            scaledInverse.t0 = (float)(double)(scaledInverse.t0 * inverseX);

            float inverseY = (float)(double)(1.0f / radii.y);
            scaledInverse.m3 = (float)(double)(inverseY * scaledInverse.m3);
            scaledInverse.m4 = (float)(double)(scaledInverse.m4 * inverseY);
            scaledInverse.m5 = (float)(double)(inverseY * scaledInverse.m5);
            scaledInverse.t1 = (float)(double)(scaledInverse.t1 * inverseY);

            float inverseZ = (float)(double)(1.0f / radii.z);
            scaledInverse.m6 = (float)(double)(inverseZ * scaledInverse.m6);
            scaledInverse.m7 = (float)(double)(scaledInverse.m7 * inverseZ);
            scaledInverse.m8 = (float)(double)(inverseZ * scaledInverse.m8);
            scaledInverse.t2 = (float)(double)(scaledInverse.t2 * inverseZ);
            return (scaledInverse, inverse);
        }

        private static void AddEllipsoidFeatureCollision(
            ref GmSurfEllipsoid ellipsoid,
            ref GmSurfMeshFace face,
            ref GmVec3 point,
            float distance,
            ref GmVec3 faceNormal,
            CHmsCollisionBuffer buffer)
        {
            float inverseDistance = (float)(double)(1.0f / distance);
            GmVec3 radial = new GmVec3(-point.x, -point.y, -point.z);
            GmVec3 normal = new GmVec3(
                (float)(double)(inverseDistance * radial.x),
                (float)(double)(radial.y * inverseDistance),
                (float)(double)(radial.z * inverseDistance));
            float scale = (float)(double)(((float)(double)(distance - 1.0f)) * inverseDistance);
            radial.x = (float)(double)(scale * radial.x);
            radial.y = (float)(double)(radial.y * scale);
            radial.z = (float)(double)(scale * radial.z);
            float planeScale = CollisionDot3(ref faceNormal, ref radial);
            GmVec3 separation = new GmVec3(
                (float)(double)(planeScale * faceNormal.x),
                (float)(double)(faceNormal.y * planeScale),
                (float)(double)(planeScale * faceNormal.z));

            ref GmCollision collision = ref buffer.AddCollision();
            FinishMeshCollision(ref collision, ref ellipsoid.@base, ref face,
                ref point, ref normal, ref separation, 0);
            collision.faceNormal = faceNormal;
        }

        /// <summary>The two output isos the game rebuilds per emitting face.</summary>
        private sealed class EllipsoidOutputIsos
        {
            public GmIso4 positionIso;
            public GmIso4 normalIso;
            public bool ready;
        }

        private static void PrepareEllipsoidOutputIsos(
            EllipsoidOutputIsos isos,
            ref GmSurfEllipsoid ellipsoid,
            ref GmIso4 inverseRelative,
            ref GmIso4 meshIso)
        {
            if (isos.ready)
            {
                return;
            }
            GmVec3 radii = ellipsoid.radii;
            float inverseX = (float)(double)(1.0f / radii.x);
            float inverseY = (float)(double)(1.0f / radii.y);
            float inverseZ = (float)(double)(1.0f / radii.z);
            GmVec3 zero = GmVec3.Zero;

            GmIso4 positionIso = IsoSetNuscaleTrans(ref radii, ref zero);
            GmIso4 invRel = inverseRelative;
            IsoMultInverse(ref positionIso, ref invRel);
            positionIso.Mult(ref meshIso);

            GmVec3 inverseRadii = new GmVec3(inverseX, inverseY, inverseZ);
            GmIso4 normalIso = IsoSetNuscaleTrans(ref inverseRadii, ref zero);
            GmIso4 invRel2 = inverseRelative;
            IsoMultInverse(ref normalIso, ref invRel2);
            normalIso.Mult(ref meshIso);

            isos.positionIso = positionIso;
            isos.normalIso = normalIso;
            isos.ready = true;
        }

        private static GmIso4 IsoSetNuscaleTrans(ref GmVec3 scale, ref GmVec3 translation)
        {
            GmIso4 iso = default(GmIso4);
            iso.t0 = translation.x;
            iso.t1 = translation.y;
            iso.t2 = translation.z;
            iso.m0 = scale.x;
            iso.m4 = scale.y;
            iso.m8 = scale.z;
            return iso;
        }

        private static void IsoMultInverse(ref GmIso4 iso, ref GmIso4 other)
        {
            GmIso4 inverse = default(GmIso4);
            inverse.SetInverse(ref other);
            iso.Mult(ref inverse);
        }

        private static void TransformEllipsoidCollisions(
            CHmsCollisionBuffer buffer,
            uint start,
            EllipsoidOutputIsos isos)
        {
            GmIso4 positionIso = isos.positionIso;
            GmIso4 normalIso = isos.normalIso;
            uint end = buffer.GetCount();
            for (uint i = start; i < end; i++)
            {
                ref GmCollision c = ref buffer.GetCollision((int)i);
                c.position.MultIso4(ref positionIso);
                CollisionHelpers.TransformDirection(ref c.normal, ref normalIso);
                float normalSquared = CollisionLengthSquared(ref c.normal);
                if (TinySquared < normalSquared)
                {
                    float length = Fp.Sqrt(normalSquared);
                    float inverseLength = (float)(double)(1.0f / length);
                    c.normal.x = (float)(double)(inverseLength * c.normal.x);
                    c.normal.y = (float)(double)(inverseLength * c.normal.y);
                    c.normal.z = (float)(double)(inverseLength * c.normal.z);
                }
                CollisionHelpers.TransformDirection(ref c.separation, ref positionIso);
            }
        }

        /// <summary>
        /// 0x008EADC0: ellipsoid contacts against a triangle mesh. The
        /// ellipsoid is scaled to the unit sphere through the scaled inverse.
        /// </summary>
        public static int GmCollisionEllipsoidMesh(
            ref LocatedGmSurf ellipsoidLocated,
            ref LocatedGmSurf meshLocated,
            CHmsCollisionBuffer buffer)
        {
            GmSurfEllipsoid ellipsoid = ellipsoidLocated.surf.AsEllipsoid();
            GmSurfMesh mesh = meshLocated.surf.AsMesh();
            GmIso4 meshIso = meshLocated.iso;
            GmIso4 relative = CollisionHelpers.RelativeIso(ref ellipsoidLocated, ref meshLocated);

            GmBoxAligned sourceBox = default(GmBoxAligned);
            sourceBox.center = GmVec3.Zero;
            sourceBox.halfExtent = ellipsoid.radii;
            GmBoxAligned meshBox = default(GmBoxAligned);
            meshBox.SetMult(ref sourceBox, ref relative);
            var (scaledInverse, inverseRelative) = MakeEllipsoidInverse(ref relative, ref ellipsoid.radii);
            MeshListNodeList? list = SelectMeshList(mesh, meshLocated.accel, ref meshBox);
            TmnfMeshPoolSlot[] pool = null;
            if (meshLocated.accel != null && meshLocated.accel.meshGrid != null)
            {
                pool = meshLocated.accel.meshGrid.pool;
            }
            EllipsoidOutputIsos outputIsos = new EllipsoidOutputIsos();

            int result = 0;
            uint nodeIndex = 0;
            uint nodeCount = list?.count ?? mesh.nodeCount;
            while (nodeIndex < nodeCount)
            {
                int nodeRef;
                uint skipCount;
                if (list.HasValue && pool != null)
                {
                    TmnfMeshPoolSlot slot = pool[(int)(list.Value.start + nodeIndex)];
                    nodeRef = (int)slot.EntryNode();
                    skipCount = slot.EntrySkip();
                }
                else
                {
                    GmSurfMeshNode n = mesh.nodes[nodeIndex];
                    nodeRef = (int)nodeIndex;
                    skipCount = n.skipCount;
                }
                // Copy the node: the query state is borrowed mutably below.
                GmSurfMeshNode node = mesh.nodes[nodeRef];
                if (!meshBox.TestInter(ref node.boxAligned))
                {
                    nodeIndex += skipCount;
                    continue;
                }
                if (node.faceIndex != uint.MaxValue)
                {
                    result |= EllipsoidMeshFace(
                        mesh, node.faceIndex, ref scaledInverse, ref inverseRelative,
                        ref meshIso, ref meshBox, ref ellipsoid, buffer, outputIsos);
                }
                nodeIndex += 1;
            }
            return result;
        }

        private static int EllipsoidMeshFace(
            GmSurfMesh mesh,
            uint faceIndex,
            ref GmIso4 scaledInverse,
            ref GmIso4 inverseRelative,
            ref GmIso4 meshIso,
            ref GmBoxAligned meshBox,
            ref GmSurfEllipsoid ellipsoid,
            CHmsCollisionBuffer buffer,
            EllipsoidOutputIsos outputIsos)
        {
            float tinySquared = TinySquared;
            float edgeClosestSquared = EdgeClosestSquared;
            GmSurfMeshFace face = mesh.faces[faceIndex];
            GmVec3[] vertices = new GmVec3[3];
            for (int i = 0; i < 3; i++)
            {
                vertices[i] = mesh.vertices[(int)face.Vertex(i)];
                vertices[i].MultIso4(ref scaledInverse);
            }

            GmVec3 edge01 = new GmVec3(
                (float)(double)(vertices[1].x - vertices[0].x),
                (float)(double)(vertices[1].y - vertices[0].y),
                (float)(double)(vertices[1].z - vertices[0].z));
            GmVec3 edge02 = new GmVec3(
                (float)(double)(vertices[2].x - vertices[0].x),
                (float)(double)(vertices[2].y - vertices[0].y),
                (float)(double)(vertices[2].z - vertices[0].z));
            GmVec3 faceNormal = new GmVec3(
                (float)(double)(((float)(double)(edge01.y * edge02.z)) - ((float)(double)(edge01.z * edge02.y))),
                (float)(double)(((float)(double)(edge02.x * edge01.z)) - ((float)(double)(edge01.x * edge02.z))),
                (float)(double)(((float)(double)(edge02.y * edge01.x)) - ((float)(double)(edge01.y * edge02.x))));
            float normalSquared = CollisionLengthSquared(ref faceNormal);
            if (!(tinySquared < normalSquared))
            {
                return 0;
            }
            float normalLength = Fp.Sqrt(normalSquared);
            float inverseNormalLength = (float)(double)(1.0f / normalLength);
            faceNormal.x = (float)(double)(inverseNormalLength * faceNormal.x);
            faceNormal.y = (float)(double)(faceNormal.y * inverseNormalLength);
            faceNormal.z = (float)(double)(inverseNormalLength * faceNormal.z);

            uint collisionStart = buffer.GetCount();
            GmVec3 negativeVertex0 = new GmVec3(-vertices[0].x, -vertices[0].y, -vertices[0].z);
            float planeDistance = CollisionDot3(ref negativeVertex0, ref faceNormal);
            if (1.0f < planeDistance || planeDistance < 0.0f)
            {
                return 0;
            }

            float radiusSquared = (float)(double)(1.0f * 1.0f);
            float sectionSquared = (float)(double)(radiusSquared - ((float)(double)(planeDistance * planeDistance)));
            float sectionRadius = Fp.Sqrt(sectionSquared);
            float negativeDistance = -planeDistance;
            GmVec3 projected = new GmVec3(
                (float)(double)(((float)(double)(faceNormal.x * negativeDistance)) + 0.0f),
                (float)(double)(((float)(double)(faceNormal.y * negativeDistance)) + 0.0f),
                (float)(double)(((float)(double)(faceNormal.z * negativeDistance)) + 0.0f));

            bool rejectFace = false;
            bool emitted = false;
            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
                int nextIndex = edgeIndex == 2 ? 0 : edgeIndex + 1;
                GmVec3 start = vertices[edgeIndex];
                GmVec3 end = vertices[nextIndex];
                GmVec3 edge = new GmVec3(
                    (float)(double)(end.x - start.x),
                    (float)(double)(end.y - start.y),
                    (float)(double)(end.z - start.z));
                float edgeSquared = CollisionLengthSquared(ref edge);
                if (tinySquared < edgeSquared)
                {
                    float edgeLength = Fp.Sqrt(edgeSquared);
                    float inverseEdgeLength = (float)(double)(1.0f / edgeLength);
                    edge.x = (float)(double)(inverseEdgeLength * edge.x);
                    edge.y = (float)(double)(edge.y * inverseEdgeLength);
                    edge.z = (float)(double)(inverseEdgeLength * edge.z);
                }

                GmVec3 side = new GmVec3(
                    (float)(double)(((float)(double)(edge.y * faceNormal.z)) - ((float)(double)(edge.z * faceNormal.y))),
                    (float)(double)(((float)(double)(edge.z * faceNormal.x)) - ((float)(double)(faceNormal.z * edge.x))),
                    (float)(double)(((float)(double)(faceNormal.y * edge.x)) - ((float)(double)(edge.y * faceNormal.x))));
                GmVec3 fromStart = new GmVec3(
                    (float)(double)(projected.x - start.x),
                    (float)(double)(projected.y - start.y),
                    (float)(double)(projected.z - start.z));
                float sideDistance = CollisionDot3(ref fromStart, ref side);
                if (sectionRadius < sideDistance)
                {
                    rejectFace = true;
                    break;
                }
                if (!(0.0f < sideDistance))
                {
                    continue;
                }

                float alongStart = CollisionDot3(ref fromStart, ref edge);
                GmVec3 closest = GmVec3.Zero;
                bool isEdge = false;
                bool isEndVertex = false;
                GmVec3 feature;
                if (0.0f <= alongStart)
                {
                    GmVec3 fromEnd = new GmVec3(
                        (float)(double)(projected.x - end.x),
                        (float)(double)(projected.y - end.y),
                        (float)(double)(projected.z - end.z));
                    float alongEnd = CollisionDot3(ref fromEnd, ref edge);
                    if (alongEnd <= 0.0f)
                    {
                        float mv = -sideDistance;
                        closest.x = (float)(double)(((float)(double)(mv * side.x)) + projected.x);
                        closest.y = (float)(double)(projected.y + ((float)(double)(side.y * mv)));
                        closest.z = (float)(double)(((float)(double)(mv * side.z)) + projected.z);
                        feature = closest;
                        isEdge = true;
                    }
                    else
                    {
                        feature = end;
                        isEndVertex = true;
                    }
                }
                else
                {
                    feature = start;
                }

                float featureSquared = CollisionLengthSquared(ref feature);
                if (isEdge)
                {
                    if (featureSquared <= edgeClosestSquared)
                    {
                        rejectFace = true;
                        break;
                    }
                }
                else if (1.0f < featureSquared || featureSquared <= tinySquared)
                {
                    rejectFace = true;
                    break;
                }

                float featureDistance = Fp.Sqrt(featureSquared);
                if (isEndVertex)
                {
                    // Retain the game's DOUBLE square root for the end vertex.
                    featureDistance = Fp.Sqrt(featureDistance);
                }
                AddEllipsoidFeatureCollision(
                    ref ellipsoid, ref face, ref feature, featureDistance, ref faceNormal, buffer);
                emitted = true;
                break;
            }

            if (!rejectFace && !emitted && 0.0f < planeDistance)
            {
                float depth = (float)(double)(planeDistance - 1.0f);
                GmVec3 separation = new GmVec3(
                    (float)(double)(depth * faceNormal.x),
                    (float)(double)(faceNormal.y * depth),
                    (float)(double)(depth * faceNormal.z));
                ref GmCollision collision = ref buffer.AddCollision();
                FinishMeshCollision(ref collision, ref ellipsoid.@base, ref face,
                    ref projected, ref faceNormal, ref separation, 1);
                collision.faceNormal = faceNormal;
                emitted = true;
            }

            if (emitted)
            {
                PrepareEllipsoidOutputIsos(outputIsos, ref ellipsoid,
                    ref inverseRelative, ref meshIso);
                TransformEllipsoidCollisions(buffer, collisionStart, outputIsos);
            }
            return emitted ? 1 : 0;
        }

        // ------------------------------------------------------------------
        // box / mesh
        // ------------------------------------------------------------------

        private static float Min3(float a, float b, float c)
        {
            float result = a;
            if (b < result) result = b;
            if (c < result) result = c;
            return result;
        }

        private static float Max3(float a, float b, float c)
        {
            float result = a;
            if (result < b) result = b;
            if (result < c) result = c;
            return result;
        }

        private static bool AxisIntervalOverlap(float a, float b, float center, float radius)
        {
            float minimum = a;
            float maximum = a;
            if (b < minimum) minimum = b;
            if (maximum < b) maximum = b;
            return minimum <= (float)(double)(radius + center) && (float)(double)(center - radius) <= maximum;
        }

        private static bool BoxTriangleOverlap(ref GmSurfBox box, GmVec3[] vertices)
        {
            float x0 = (float)(double)(vertices[0].x - box.center.x);
            float x1 = (float)(double)(vertices[1].x - box.center.x);
            float x2 = (float)(double)(vertices[2].x - box.center.x);
            if (!(Min3(x0, x1, x2) <= box.halfExtent.x)
                || !(-box.halfExtent.x <= Max3(x0, x1, x2)))
            {
                return false;
            }

            float y0 = (float)(double)(vertices[0].y - box.center.y);
            float y1 = (float)(double)(vertices[1].y - box.center.y);
            float y2 = (float)(double)(vertices[2].y - box.center.y);
            if (!(Min3(y0, y1, y2) <= box.halfExtent.y)
                || !(-box.halfExtent.y <= Max3(y0, y1, y2)))
            {
                return false;
            }

            float z0 = (float)(double)(vertices[0].z - box.center.z);
            float z1 = (float)(double)(vertices[1].z - box.center.z);
            float z2 = (float)(double)(vertices[2].z - box.center.z);
            if (!(Min3(z0, z1, z2) <= box.halfExtent.z)
                || !(-box.halfExtent.z <= Max3(z0, z1, z2)))
            {
                return false;
            }

            GmVec3 edge0 = new GmVec3((float)(double)(x1 - x0), (float)(double)(y1 - y0), (float)(double)(z1 - z0));
            GmVec3 edge1 = new GmVec3((float)(double)(x2 - x1), (float)(double)(y2 - y1), (float)(double)(z2 - z1));
            GmVec3 triangleNormal = new GmVec3(
                (float)(double)(((float)(double)(edge0.y * edge1.z)) - ((float)(double)(edge0.z * edge1.y))),
                (float)(double)(((float)(double)(edge1.x * edge0.z)) - ((float)(double)(edge0.x * edge1.z))),
                (float)(double)(((float)(double)(edge1.y * edge0.x)) - ((float)(double)(edge1.x * edge0.y))));
            float plane = -((float)(double)(((float)(double)(((float)(double)(triangleNormal.x * x0)) + ((float)(double)(triangleNormal.y * y0))))
                + ((float)(double)(triangleNormal.z * z0))));
            GmVec3 positive = GmVec3.Zero;
            GmVec3 negative = GmVec3.Zero;
            float[] extents = { box.halfExtent.x, box.halfExtent.y, box.halfExtent.z };
            float[] normal = { triangleNormal.x, triangleNormal.y, triangleNormal.z };
            for (int axis = 0; axis < 3; axis++)
            {
                if (normal[axis] <= 0.0f)
                {
                    SetAxis(ref positive, axis, extents[axis]);
                    SetAxis(ref negative, axis, -extents[axis]);
                }
                else
                {
                    SetAxis(ref positive, axis, -extents[axis]);
                    SetAxis(ref negative, axis, extents[axis]);
                }
            }
            float positivePlane = (float)(double)(((float)(double)(((float)(double)(((float)(double)(positive.x * triangleNormal.x))
                + ((float)(double)(positive.y * triangleNormal.y))))
                + ((float)(double)(positive.z * triangleNormal.z)))) + plane);
            float negativePlane = (float)(double)(((float)(double)(((float)(double)(((float)(double)(negative.x * triangleNormal.x))
                + ((float)(double)(negative.y * triangleNormal.y))))
                + ((float)(double)(negative.z * triangleNormal.z)))) + plane);
            if (!(positivePlane <= 0.0f) || !(0.0f <= negativePlane))
            {
                return false;
            }

            GmVec3[] edges = { edge0, edge1, new GmVec3((float)(double)(x0 - x2), (float)(double)(y0 - y2), (float)(double)(z0 - z2)) };
            GmVec3[] centered = { new GmVec3(x0, y0, z0), new GmVec3(x1, y1, z1), new GmVec3(x2, y2, z2) };

            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
                GmVec3 edge = edges[edgeIndex];
                GmVec3 @base = centered[edgeIndex];
                GmVec3 opposite = centered[(edgeIndex + 2) % 3];
                float[] projections = new float[2];
                float radius;

                projections[0] = (float)(double)(((float)(double)(edge.z * @base.y)) - ((float)(double)(edge.y * @base.z)));
                projections[1] = (float)(double)(((float)(double)(edge.z * opposite.y)) - ((float)(double)(edge.y * opposite.z)));
                radius = (float)(double)(((float)(double)(box.halfExtent.y * Math.Abs(edge.z)))
                    + ((float)(double)(box.halfExtent.z * Math.Abs(edge.y))));
                if (!AxisIntervalOverlap(projections[0], projections[1], 0.0f, radius))
                {
                    return false;
                }

                projections[0] = (float)(double)(((float)(double)(edge.x * @base.z)) - ((float)(double)(edge.z * @base.x)));
                projections[1] = (float)(double)(((float)(double)(edge.x * opposite.z)) - ((float)(double)(edge.z * opposite.x)));
                radius = (float)(double)(((float)(double)(box.halfExtent.z * Math.Abs(edge.x)))
                    + ((float)(double)(box.halfExtent.x * Math.Abs(edge.z))));
                if (!AxisIntervalOverlap(projections[0], projections[1], 0.0f, radius))
                {
                    return false;
                }

                projections[0] = (float)(double)(((float)(double)(edge.y * @base.x)) - ((float)(double)(edge.x * @base.y)));
                projections[1] = (float)(double)(((float)(double)(edge.y * opposite.x)) - ((float)(double)(edge.x * opposite.y)));
                radius = (float)(double)(((float)(double)(box.halfExtent.y * Math.Abs(edge.x)))
                    + ((float)(double)(box.halfExtent.x * Math.Abs(edge.y))));
                if (!AxisIntervalOverlap(projections[0], projections[1], 0.0f, radius))
                {
                    return false;
                }
            }
            return true;
        }

        private static void SetAxis(ref GmVec3 v, int axis, float value)
        {
            switch (axis)
            {
                case 0: v.x = value; break;
                case 1: v.y = value; break;
                default: v.z = value; break;
            }
        }

        /// <summary>0x008F5200: oriented box vs triangle mesh (first hit only).</summary>
        public static int GmCollisionBoxMesh(
            ref LocatedGmSurf boxLocated,
            ref LocatedGmSurf meshLocated,
            CHmsCollisionBuffer buffer)
        {
            GmSurfBox box = boxLocated.surf.AsBox();
            GmSurfMesh mesh = meshLocated.surf.AsMesh();
            GmIso4 relative = CollisionHelpers.RelativeIso(ref boxLocated, ref meshLocated);

            GmBoxAligned sourceBox = default(GmBoxAligned);
            sourceBox.center = box.center;
            sourceBox.halfExtent = box.halfExtent;
            GmBoxAligned meshBox = default(GmBoxAligned);
            meshBox.SetMult(ref sourceBox, ref relative);

            uint nodeIndex = 0;
            while (nodeIndex < mesh.nodeCount)
            {
                GmSurfMeshNode node = mesh.nodes[nodeIndex];
                if (!meshBox.TestInter(ref node.boxAligned))
                {
                    nodeIndex += node.skipCount;
                    continue;
                }
                if (node.faceIndex == uint.MaxValue)
                {
                    nodeIndex += 1;
                    continue;
                }

                GmSurfMeshFace face = mesh.faces[node.faceIndex];
                GmVec3[] vertices = new GmVec3[3];
                for (int i = 0; i < 3; i++)
                {
                    GmVec3 source = mesh.vertices[(int)face.Vertex(i)];
                    vertices[i].x = (float)(double)(source.x - relative.t0);
                    vertices[i].y = (float)(double)(source.y - relative.t1);
                    vertices[i].z = (float)(double)(source.z - relative.t2);
                    GmMat3 relRot = default(GmMat3);
                    relRot.m0 = relative.m0; relRot.m1 = relative.m1; relRot.m2 = relative.m2;
                    relRot.m3 = relative.m3; relRot.m4 = relative.m4; relRot.m5 = relative.m5;
                    relRot.m6 = relative.m6; relRot.m7 = relative.m7; relRot.m8 = relative.m8;
                    vertices[i].MultTranspose(ref relRot);
                }
                if (!BoxTriangleOverlap(ref box, vertices))
                {
                    nodeIndex += 1;
                    continue;
                }

                ref GmCollision c = ref buffer.AddCollision();
                c.separation.x = 0.0f;
                c.separation.y = 0.0f;
                c.separation.z = 0.0f;
                GmVec3 position = mesh.vertices[face.vertex0];
                GmIso4 meshIso = meshLocated.iso;
                position.MultIso4(ref meshIso);
                c.position = position;
                c.normal = face.normal;
                CollisionHelpers.TransformDirection(ref c.normal, ref meshIso);
                c.material1 = box.@base.materialIndex;
                c.material2 = face.materialIndex;
                return 1;
            }
            return 0;
        }

        // ------------------------------------------------------------------
        // dispatch
        // ------------------------------------------------------------------

        /// <summary>The shape dispatch table (only the mesh handlers populated).</summary>
        public sealed class CollisionShapeDispatch
        {
            public NarrowphasePairHandler SphereBox;
            public NarrowphasePairHandler SphereEllipsoid;
            public NarrowphasePairHandler SpherePolygon;
            public NarrowphasePairHandler EllipsoidPolygon;
            public NarrowphasePairHandler SphereMesh;
            public NarrowphasePairHandler EllipsoidMesh;
            public NarrowphasePairHandler BoxBox;
            public NarrowphasePairHandler BoxMesh;
            public NarrowphasePairHandler MeshMesh;

            public static CollisionShapeDispatch Default()
            {
                return new CollisionShapeDispatch
                {
                    SphereBox = null,
                    SphereEllipsoid = null,
                    SpherePolygon = null,
                    EllipsoidPolygon = null,
                    SphereMesh = GmCollisionSphereMesh,
                    EllipsoidMesh = GmCollisionEllipsoidMesh,
                    BoxBox = null,
                    BoxMesh = GmCollisionBoxMesh,
                    MeshMesh = null,
                };
            }
        }

        public delegate int NarrowphasePairHandler(
            ref LocatedGmSurf first,
            ref LocatedGmSurf second,
            CHmsCollisionBuffer buffer);

        private static bool IsResolvedPair(byte type1, byte type2)
        {
            return (type1 == GmSurfTypes.GmSurfSphere && type2 == GmSurfTypes.GmSurfBox)
                || (type1 == GmSurfTypes.GmSurfSphere && type2 == GmSurfTypes.GmSurfEllipsoid)
                || (type1 == GmSurfTypes.GmSurfSphere && type2 == GmSurfTypes.GmSurfPolygon)
                || (type1 == GmSurfTypes.GmSurfEllipsoid && type2 == GmSurfTypes.GmSurfPolygon)
                || (type1 == GmSurfTypes.GmSurfSphere && type2 == GmSurfTypes.GmSurfMesh)
                || (type1 == GmSurfTypes.GmSurfEllipsoid && type2 == GmSurfTypes.GmSurfMesh)
                || (type1 == GmSurfTypes.GmSurfBox && type2 == GmSurfTypes.GmSurfBox)
                || (type1 == GmSurfTypes.GmSurfBox && type2 == GmSurfTypes.GmSurfMesh)
                || (type1 == GmSurfTypes.GmSurfMesh && type2 == GmSurfTypes.GmSurfMesh);
        }

        private static NarrowphasePairHandler ResolvedHandler(
            byte type1, byte type2, CollisionShapeDispatch runtime)
        {
            if (type1 == GmSurfTypes.GmSurfSphere && type2 == GmSurfTypes.GmSurfBox) return runtime.SphereBox;
            if (type1 == GmSurfTypes.GmSurfSphere && type2 == GmSurfTypes.GmSurfEllipsoid) return runtime.SphereEllipsoid;
            if (type1 == GmSurfTypes.GmSurfSphere && type2 == GmSurfTypes.GmSurfPolygon) return runtime.SpherePolygon;
            if (type1 == GmSurfTypes.GmSurfEllipsoid && type2 == GmSurfTypes.GmSurfPolygon) return runtime.EllipsoidPolygon;
            if (type1 == GmSurfTypes.GmSurfSphere && type2 == GmSurfTypes.GmSurfMesh) return runtime.SphereMesh;
            if (type1 == GmSurfTypes.GmSurfEllipsoid && type2 == GmSurfTypes.GmSurfMesh) return runtime.EllipsoidMesh;
            if (type1 == GmSurfTypes.GmSurfBox && type2 == GmSurfTypes.GmSurfBox) return runtime.BoxBox;
            if (type1 == GmSurfTypes.GmSurfBox && type2 == GmSurfTypes.GmSurfMesh) return runtime.BoxMesh;
            if (type1 == GmSurfTypes.GmSurfMesh && type2 == GmSurfTypes.GmSurfMesh) return runtime.MeshMesh;
            return null;
        }

        /// <summary>Negates a collision's normal/separation and swaps materials.</summary>
        public static void GmCollisionNeg(ref GmCollision collision)
        {
            collision.normal.x = -collision.normal.x;
            collision.normal.y = -collision.normal.y;
            collision.normal.z = -collision.normal.z;
            ushort material = collision.material1;
            collision.material1 = collision.material2;
            collision.separation.x = -collision.separation.x;
            collision.material2 = material;
            collision.separation.y = -collision.separation.y;
            collision.separation.z = -collision.separation.z;
            collision.faceNormal.x = -collision.faceNormal.x;
            collision.faceNormal.y = -collision.faceNormal.y;
            collision.faceNormal.z = -collision.faceNormal.z;
        }

        /// <summary>0x008E8890: dispatches a geometry pair, fixes reversed output.</summary>
        public static int GmSurfComputeCollision(
            ref LocatedGmSurf surf1,
            ref LocatedGmSurf surf2,
            CHmsCollisionBuffer buffer,
            CollisionShapeDispatch runtime)
        {
            LocatedGmSurf first = surf1;
            LocatedGmSurf second = surf2;
            bool reversed = false;
            if (surf1.surf.SurfType() > surf2.surf.SurfType())
            {
                first = surf2;
                second = surf1;
                reversed = true;
            }

            byte type1 = first.surf.SurfType();
            byte type2 = second.surf.SurfType();
            uint start = buffer.GetCount();
            int result;
            if (type1 == GmSurfTypes.GmSurfSphere && type2 == GmSurfTypes.GmSurfSphere)
            {
                result = GmCollisionSphereSphere(ref first, ref second, buffer);
            }
            else if (!IsResolvedPair(type1, type2))
            {
                result = 0;
            }
            else
            {
                NarrowphasePairHandler handler = ResolvedHandler(type1, type2, runtime);
                if (handler == null)
                {
                    throw new InvalidOperationException("tmnf: no collision handler for shape pair");
                }
                result = handler(ref first, ref second, buffer);
            }
            if (result == 0 || !reversed)
            {
                return result;
            }

            uint end = buffer.GetCount();
            for (uint i = start; i < end; i++)
            {
                ref GmCollision c = ref buffer.GetCollision((int)i);
                GmCollisionNeg(ref c);
            }
            return 1;
        }

        /// <summary>0x00537150: dispatch + material remap (no accel).</summary>
        public static int CplugSurfaceComputeCollision(
            CPlugSurface surface1, ref GmIso4 iso1,
            CPlugSurface surface2, ref GmIso4 iso2,
            CHmsCollisionBuffer buffer,
            CollisionShapeDispatch runtime)
        {
            return ComputeSurfaceCollision(
                surface1, ref iso1, surface2, ref iso2, null, buffer, runtime);
        }

        /// <summary>0x00537150: dispatch + material remap (optional accel).</summary>
        public static int ComputeSurfaceCollision(
            CPlugSurface surface1, ref GmIso4 iso1,
            CPlugSurface surface2, ref GmIso4 iso2,
            TmnfMeshQueryAccel accel2,
            CHmsCollisionBuffer buffer,
            CollisionShapeDispatch runtime)
        {
            uint start = buffer.GetCount();
            LocatedGmSurf surf1 = new LocatedGmSurf
            {
                surf = surface1.geom,
                iso = iso1,
                isLocated = 1,
                accel = null,
            };
            LocatedGmSurf surf2 = new LocatedGmSurf
            {
                surf = surface2.geom,
                iso = iso2,
                isLocated = 1,
                accel = accel2,
            };
            if (GmSurfComputeCollision(ref surf1, ref surf2, buffer, runtime) == 0)
            {
                return 0;
            }

            uint end = buffer.GetCount();
            for (uint i = start; i < end; i++)
            {
                ref GmCollision collision = ref buffer.GetCollision((int)i);
                if (collision.material1 >= surface1.materialIds.Length
                    || collision.material2 >= surface2.materialIds.Length)
                {
                    throw new InvalidOperationException("tmnf: collision material index out of range");
                }
                collision.material1 = surface1.materialIds[collision.material1];
                collision.material2 = surface2.materialIds[collision.material2];
            }
            return 1;
        }

        // ------------------------------------------------------------------
        // static grid locate
        // ------------------------------------------------------------------

        /// <summary>
        /// A query is served from the grid when its box lies inside the region
        /// its cell's list was built for and the surface is a sphere or
        /// ellipsoid no larger than the region expansion.
        /// </summary>
        public static uint? StaticGridLocate(
            TmnfStaticGrid grid,
            ref GmBoxAligned box,
            CPlugSurface surface)
        {
            GmVec3 shapeRadii;
            if (surface.geom is GmSurfGeomEllipsoid e)
            {
                shapeRadii = e.shape.radii;
            }
            else if (surface.geom is GmSurfGeomSphere s)
            {
                shapeRadii = new GmVec3(s.shape.radius, s.shape.radius, s.shape.radius);
            }
            else
            {
                return null;
            }
            float[] radii = { shapeRadii.x, shapeRadii.y, shapeRadii.z };
            float[] c = { box.center.x, box.center.y, box.center.z };
            float[] h = { box.halfExtent.x, box.halfExtent.y, box.halfExtent.z };
            double inner = grid.expand - grid.margin;
            uint[] index = new uint[3];
            for (int a = 0; a < 3; a++)
            {
                if (!((double)radii[a] <= grid.expand))
                {
                    return null;
                }
                double offset = (double)c[a] - grid.origin[a];
                if (!(offset >= 0.0 && offset < (double)grid.dims[a] * grid.cellSize))
                {
                    return null;
                }
                index[a] = (uint)(offset * grid.invCellSize);
                if (index[a] >= grid.dims[a])
                {
                    return null;
                }
                double lo = grid.origin[a] + (double)index[a] * grid.cellSize;
                double hi = lo + grid.cellSize;
                if (!(((double)c[a] - (double)h[a]) >= lo - inner
                    && ((double)c[a] + (double)h[a]) <= hi + inner))
                {
                    return null;
                }
            }
            return (index[2] * grid.dims[1] + index[1]) * grid.dims[0] + index[0];
        }
    }
}
