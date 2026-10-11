using System;
using System;
using NUnit.Framework;
using Tmnf.Sim;

namespace Tmnf.Sim.Tests
{
    /// <summary>Golden suites for buffer (compare + MSVC qsort) and collision kernels.</summary>
    [TestFixture]
    public class CollisionGoldenTests
    {
        private static SHmsPhysicalCollision ReadRecord(ByteReader r)
        {
            SHmsPhysicalCollision rec = default(SHmsPhysicalCollision);
            rec.corpus1 = r.U32();
            rec.tree1 = r.U32();
            rec.corpus2 = r.U32();
            rec.tree2 = r.U32();
            rec.collision.separation = GmGoldenTests.ReadVec3(r);
            rec.collision.normal = GmGoldenTests.ReadVec3(r);
            rec.collision.position = GmGoldenTests.ReadVec3(r);
            rec.collision.material1 = r.U16();
            rec.collision.material2 = r.U16();
            rec.collision.flags = r.U32();
            rec.collision.faceNormal = GmGoldenTests.ReadVec3(r);
            rec.material = r.U32();
            return rec;
        }

        private static void WriteRecord(System.Collections.Generic.List<byte> o, ref SHmsPhysicalCollision rec)
        {
            ByteWriter.U32(o, rec.corpus1);
            ByteWriter.U32(o, rec.tree1);
            ByteWriter.U32(o, rec.corpus2);
            ByteWriter.U32(o, rec.tree2);
            GmGoldenTests.WriteVec3To(o, rec.collision.separation);
            GmGoldenTests.WriteVec3To(o, rec.collision.normal);
            GmGoldenTests.WriteVec3To(o, rec.collision.position);
            ByteWriter.U16(o, rec.collision.material1);
            ByteWriter.U16(o, rec.collision.material2);
            ByteWriter.U32(o, rec.collision.flags);
            GmGoldenTests.WriteVec3To(o, rec.collision.faceNormal);
            ByteWriter.U32(o, rec.material);
        }

        private static void WriteBuffer(System.Collections.Generic.List<byte> o, CFastBufferShmsPhysicalCollision buf)
        {
            ByteWriter.U32(o, buf.Count());
            for (int i = 0; i < buf.count; i++)
            {
                SHmsPhysicalCollision rec = buf.At(i);
                WriteRecord(o, ref rec);
            }
        }

        [Test]
        public void BufferCompareTest()
        {
            GoldenRunner.RunSuite("buffer/compare.tgn", input =>
            {
                var r = new ByteReader(input);
                SHmsPhysicalCollision a = ReadRecord(r);
                SHmsPhysicalCollision b = ReadRecord(r);
                return BitConverter.GetBytes(BufferCompare.ShmsPhysicalCollisionCompare(ref a, ref b));
            });
        }

        [Test]
        public void BufferQsortTest()
        {
            GoldenRunner.RunSuite("buffer/qsort.tgn", input =>
            {
                var r = new ByteReader(input);
                uint n = r.U32();
                var buf = new CFastBufferShmsPhysicalCollision();
                for (int i = 0; i < (int)n; i++)
                {
                    buf.Push(ReadRecord(r));
                }
                buf.Qsort(BufferCompare.ShmsPhysicalCollisionCompare);
                var o = new System.Collections.Generic.List<byte>();
                WriteBuffer(o, buf);
                return o.ToArray();
            });
        }

        [Test]
        public void EdgePrepare()
        {
            GoldenRunner.RunSuite("collision/edge_prepare.tgn", input =>
            {
                var r = new ByteReader(input);
                GmVec3 vertex = GmGoldenTests.ReadVec3(r);
                GmVec3 next = GmGoldenTests.ReadVec3(r);
                GmVec3 normal = GmGoldenTests.ReadVec3(r);
                TmnfSphereMeshEdge edge = CollisionHelpers.TmnfSphereMeshEdgePrepare(ref vertex, ref next, ref normal);
                var o = new System.Collections.Generic.List<byte>();
                GmGoldenTests.WriteVec3To(o, edge.direction);
                GmGoldenTests.WriteVec3To(o, edge.side);
                return o.ToArray();
            });
        }

        [Test]
        public void SphereMesh()
        {
            GoldenRunner.RunSuite("collision/sphere_mesh.tgn", input =>
            {
                var r = new ByteReader(input);
                CPlugSurface surf1 = ReadSurface(r);
                GmIso4 iso1 = GmGoldenTests.ReadIso4(r);
                CPlugSurface surf2 = ReadSurface(r);
                GmIso4 iso2 = GmGoldenTests.ReadIso4(r);
                LocatedGmSurf sphereLocated = new LocatedGmSurf
                {
                    surf = surf1.geom, iso = iso1, isLocated = 1, accel = null,
                };
                LocatedGmSurf meshLocated = new LocatedGmSurf
                {
                    surf = surf2.geom, iso = iso2, isLocated = 1, accel = null,
                };
                CHmsCollisionBuffer buffer = new CHmsCollisionBuffer();
                int result = Narrowphase.GmCollisionSphereMesh(ref sphereLocated, ref meshLocated, buffer);
                var o = new System.Collections.Generic.List<byte>();
                ByteWriter.I32(o, result);
                WriteBuffer(o, buffer.collisions);
                return o.ToArray();
            });
        }

        [Test]
        public void EllipsoidMesh()
        {
            GoldenRunner.RunSuite("collision/ellipsoid_mesh.tgn", input =>
            {
                var r = new ByteReader(input);
                CPlugSurface surf1 = ReadSurface(r);
                GmIso4 iso1 = GmGoldenTests.ReadIso4(r);
                CPlugSurface surf2 = ReadSurface(r);
                GmIso4 iso2 = GmGoldenTests.ReadIso4(r);
                LocatedGmSurf ellipsoidLocated = new LocatedGmSurf
                {
                    surf = surf1.geom, iso = iso1, isLocated = 1, accel = null,
                };
                LocatedGmSurf meshLocated = new LocatedGmSurf
                {
                    surf = surf2.geom, iso = iso2, isLocated = 1, accel = null,
                };
                CHmsCollisionBuffer buffer = new CHmsCollisionBuffer();
                int result = Narrowphase.GmCollisionEllipsoidMesh(ref ellipsoidLocated, ref meshLocated, buffer);
                var o = new System.Collections.Generic.List<byte>();
                ByteWriter.I32(o, result);
                WriteBuffer(o, buffer.collisions);
                return o.ToArray();
            });
        }

        [Test]
        public void ComputeSurface()
        {
            GoldenRunner.RunSuite("collision/compute_surface.tgn", input =>
            {
                var r = new ByteReader(input);
                bool reversed = r.U8() != 0;
                CPlugSurface shape = ReadSurface(r);
                GmIso4 iso1 = GmGoldenTests.ReadIso4(r);
                CPlugSurface mesh = ReadSurface(r);
                GmIso4 iso2 = GmGoldenTests.ReadIso4(r);
                CHmsCollisionBuffer buffer = new CHmsCollisionBuffer();
                int result;
                if (reversed)
                {
                    result = Narrowphase.CplugSurfaceComputeCollision(mesh, ref iso2, shape, ref iso1, buffer,
                        Narrowphase.CollisionShapeDispatch.Default());
                }
                else
                {
                    result = Narrowphase.CplugSurfaceComputeCollision(shape, ref iso1, mesh, ref iso2, buffer,
                        Narrowphase.CollisionShapeDispatch.Default());
                }
                var o = new System.Collections.Generic.List<byte>();
                ByteWriter.I32(o, result);
                WriteBuffer(o, buffer.collisions);
                return o.ToArray();
            });
        }

        [Test]
        public void Merge()
        {
            GoldenRunner.RunSuite("collision/merge.tgn", input =>
            {
                var r = new ByteReader(input);
                SHmsSphereBufferContact sphere = new SHmsSphereBufferContact();
                ReadIntoBuffer(r, sphere.@base);
                sphere.active = r.U32();
                CHmsCollisionBuffer destination = new CHmsCollisionBuffer();
                ReadIntoBuffer(r, destination);
                Narrowphase.ShmsSphereBufferContactMergeAndAddToCollisions(sphere, destination);
                var o = new System.Collections.Generic.List<byte>();
                WriteBuffer(o, destination.collisions);
                ByteWriter.U32(o, sphere.active);
                return o.ToArray();
            });
        }

        [Test]
        public void GridLocate()
        {
            // The fixture track loads through the real loader.
            string tmp = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "tmnf-golden-selftest");
            System.IO.Directory.CreateDirectory(tmp);
            string trackPath = System.IO.Path.Combine(tmp, "plain.tmnftrack");
            if (!System.IO.File.Exists(trackPath))
            {
                byte[] fixture = System.IO.File.ReadAllBytes(
                    GoldenRunner.GoldenPath("fixtures/plain.tmnftrack"));
                System.IO.File.WriteAllBytes(trackPath, fixture);
            }
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoad(trackPath, null);
            GoldenRunner.RunSuite("collision/grid_locate.tgn", input =>
            {
                var r = new ByteReader(input);
                GmBoxAligned box = default(GmBoxAligned);
                box.center = GmGoldenTests.ReadVec3(r);
                box.halfExtent = GmGoldenTests.ReadVec3(r);
                CPlugSurface surface = ReadSurface(r);
                uint? cell = Narrowphase.StaticGridLocate(track.grid, ref box, surface);
                return BitConverter.GetBytes(cell.HasValue ? unchecked((int)cell.Value) : -1);
            });
        }

        [Test]
        public void SphereMeshAccel()
        {
            string tmp = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "tmnf-golden-selftest");
            System.IO.Directory.CreateDirectory(tmp);
            string trackPath = System.IO.Path.Combine(tmp, "plain.tmnftrack");
            if (!System.IO.File.Exists(trackPath))
            {
                byte[] fixture = System.IO.File.ReadAllBytes(
                    GoldenRunner.GoldenPath("fixtures/plain.tmnftrack"));
                System.IO.File.WriteAllBytes(trackPath, fixture);
            }
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoad(trackPath, null);
            GoldenRunner.RunSuite("collision/sphere_mesh_accel.tgn", input =>
            {
                var r = new ByteReader(input);
                int entryIndex = (int)r.U32();
                CPlugSurface surf1 = ReadSurface(r);
                GmIso4 iso1 = GmGoldenTests.ReadIso4(r);
                r.U8(); // accel mode byte (always 1 in this suite)
                HmsStaticCollisionEntry entry = track.entries[entryIndex];
                CPlugSurface useSurface = entry.surface >= 0 ? track.surfaces[entry.surface] : track.surfaces[0];
                int? meshGridIndex = track.grid.entryMeshGrids[entryIndex];
                TmnfMeshQueryAccel accel = new TmnfMeshQueryAccel();
                accel.inverseIso = track.grid.entryInverseIsos[entryIndex];
                accel.meshGrid = meshGridIndex.HasValue ? track.grid.meshGrids[meshGridIndex.Value] : null;
                GmIso4 iso2 = entry.iso;
                LocatedGmSurf sphereLocated = new LocatedGmSurf
                {
                    surf = surf1.geom, iso = iso1, isLocated = 1, accel = accel,
                };
                LocatedGmSurf meshLocated = new LocatedGmSurf
                {
                    surf = useSurface.geom, iso = iso2, isLocated = 1, accel = null,
                };
                var o = new System.Collections.Generic.List<byte>();
                CHmsCollisionBuffer buffer = new CHmsCollisionBuffer();
                int result = Narrowphase.GmCollisionSphereMesh(ref sphereLocated, ref meshLocated, buffer);
                ByteWriter.I32(o, result);
                WriteBuffer(o, buffer.collisions);
                CHmsCollisionBuffer buffer2 = new CHmsCollisionBuffer();
                LocatedGmSurf sphereLocated2 = new LocatedGmSurf
                {
                    surf = surf1.geom, iso = iso1, isLocated = 1, accel = null,
                };
                int result2 = Narrowphase.GmCollisionSphereMesh(ref sphereLocated2, ref meshLocated, buffer2);
                ByteWriter.I32(o, result2);
                WriteBuffer(o, buffer2.collisions);
                return o.ToArray();
            });
        }

        // ------------------------------------------------------------------
        // serialization helpers
        // ------------------------------------------------------------------

        private static void ReadIntoBuffer(ByteReader r, CHmsCollisionBuffer buffer)
        {
            uint count = r.U32();
            for (int i = 0; i < (int)count; i++)
            {
                buffer.collisions.Push(ReadRecord(r));
            }
        }

        private static LocatedGmSurf ReadSphere(ByteReader r)
        {
            byte kind = r.U8();
            if (kind != 0)
            {
                throw new InvalidDataException("expected sphere");
            }
            float radius = r.F32();
            ushort material = r.U16();
            GmSurfSphere shape = default(GmSurfSphere);
            shape.@base.materialIndex = material;
            shape.@base.surfType = GmSurfTypes.GmSurfSphere;
            shape.radius = radius;
            CPlugSurface surface = new CPlugSurface();
            surface.geom = new GmSurfGeomSphere { shape = shape };
            surface.materialIds = new byte[0];
            surface.materialCount = 0;
            return new LocatedGmSurf
            {
                surf = surface.geom,
                iso = GmGoldenTests.ReadIso4(r),
                isLocated = 1,
                accel = null,
            };
        }

        private static LocatedGmSurf ReadEllipsoid(ByteReader r)
        {
            byte kind = r.U8();
            if (kind != 1)
            {
                throw new InvalidDataException("expected ellipsoid");
            }
            GmVec3 radii = GmGoldenTests.ReadVec3(r);
            ushort material = r.U16();
            GmSurfEllipsoid shape = default(GmSurfEllipsoid);
            shape.@base.materialIndex = material;
            shape.@base.surfType = GmSurfTypes.GmSurfEllipsoid;
            shape.radii = radii;
            CPlugSurface surface = new CPlugSurface();
            surface.geom = new GmSurfGeomEllipsoid { shape = shape };
            surface.materialIds = new byte[0];
            surface.materialCount = 0;
            return new LocatedGmSurf
            {
                surf = surface.geom,
                iso = GmGoldenTests.ReadIso4(r),
                isLocated = 1,
                accel = null,
            };
        }

        private static LocatedGmSurf ReadMesh(ByteReader r)
        {
            byte kind = r.U8();
            if (kind != 7)
            {
                throw new InvalidDataException("expected mesh");
            }
            GmSurfMesh mesh = ReadMeshData(r);
            return new LocatedGmSurf
            {
                surf = new GmSurfGeomMesh { mesh = mesh },
                iso = GmGoldenTests.ReadIso4(r),
                isLocated = 1,
                accel = null,
            };
        }

        private static GmSurfMesh ReadMeshData(ByteReader r)
        {
            GmSurfMesh mesh = new GmSurfMesh();
            mesh.@base.surfType = GmSurfTypes.GmSurfMesh;
            uint vertexCount = r.U32();
            mesh.vertexCount = vertexCount;
            mesh.vertices = new GmVec3[vertexCount];
            for (int i = 0; i < (int)vertexCount; i++)
            {
                mesh.vertices[i] = GmGoldenTests.ReadVec3(r);
            }
            uint faceCount = r.U32();
            mesh.faceCount = faceCount;
            mesh.faces = new GmSurfMeshFace[faceCount];
            for (int i = 0; i < (int)faceCount; i++)
            {
                GmSurfMeshFace f = default(GmSurfMeshFace);
                f.normal = GmGoldenTests.ReadVec3(r);
                f.reserved0c = r.U32();
                f.vertex0 = r.U32();
                f.vertex1 = r.U32();
                f.vertex2 = r.U32();
                f.materialIndex = r.U16();
                f.reserved1e = r.U16();
                mesh.faces[i] = f;
            }
            uint nodeCount = r.U32();
            mesh.nodeCount = nodeCount;
            mesh.nodes = new GmSurfMeshNode[nodeCount];
            for (int i = 0; i < (int)nodeCount; i++)
            {
                GmSurfMeshNode n = default(GmSurfMeshNode);
                n.skipCount = r.U32();
                n.boxAligned.center = GmGoldenTests.ReadVec3(r);
                n.boxAligned.halfExtent = GmGoldenTests.ReadVec3(r);
                n.faceIndex = r.U32();
                mesh.nodes[i] = n;
            }
            return mesh;
        }

        private static CPlugSurface ReadSurface(ByteReader r)
        {
            byte kind = r.U8();
            CPlugSurface s = new CPlugSurface();
            if (kind == 0)
            {
                float radius = r.F32();
                ushort material = r.U16();
                GmSurfSphere shape = default(GmSurfSphere);
                shape.@base.materialIndex = material;
                shape.@base.surfType = GmSurfTypes.GmSurfSphere;
                shape.radius = radius;
                s.geom = new GmSurfGeomSphere { shape = shape };
            }
            else if (kind == 1)
            {
                GmVec3 radii = GmGoldenTests.ReadVec3(r);
                ushort material = r.U16();
                GmSurfEllipsoid shape = default(GmSurfEllipsoid);
                shape.@base.materialIndex = material;
                shape.@base.surfType = GmSurfTypes.GmSurfEllipsoid;
                shape.radii = radii;
                s.geom = new GmSurfGeomEllipsoid { shape = shape };
            }
            else if (kind == 7)
            {
                s.geom = new GmSurfGeomMesh { mesh = ReadMeshData(r) };
            }
            else
            {
                throw new InvalidDataException($"unsupported surface kind {kind}");
            }
            uint materialCount = r.U32();
            s.materialIds = r.Bytes((int)materialCount);
            s.materialCount = materialCount;
            return s;
        }

        private class InvalidDataException : Exception
        {
            public InvalidDataException(string m) : base(m) { }
        }
    }
}
