using System;

namespace Tmnf.Sim
{
    /// <summary>Center/half-extent AABB (24 bytes).</summary>
    [Serializable]
    public struct GmBoxAligned
    {
        public GmVec3 center;
        public GmVec3 halfExtent;

        /// <summary>Tests overlap of two AABBs (z, y, x order with early exits).</summary>
        public bool TestInter(ref GmBoxAligned other)
        {
            float delta = (float)(double)(other.center.z - center.z);
            float absoluteDelta = Math.Abs(delta);
            float extent = (float)(double)(other.halfExtent.z + halfExtent.z);
            if (!(absoluteDelta <= extent))
            {
                return false;
            }

            delta = (float)(double)(other.center.y - center.y);
            absoluteDelta = Math.Abs(delta);
            extent = (float)(double)(other.halfExtent.y + halfExtent.y);
            if (!(absoluteDelta <= extent))
            {
                return false;
            }

            delta = (float)(double)(other.center.x - center.x);
            absoluteDelta = Math.Abs(delta);
            extent = (float)(double)(other.halfExtent.x + halfExtent.x);
            return absoluteDelta <= extent;
        }

        /// <summary>Transforms an AABB by an affine isometry (abs of the rotation).</summary>
        public void SetMult(ref GmBoxAligned source, ref GmIso4 iso)
        {
            center.x = (float)(double)(((float)(double)(((float)(double)(((float)(double)(source.center.x * iso.m0)) + ((float)(double)(iso.m1 * source.center.y))))
                + ((float)(double)(iso.m2 * source.center.z)))) + iso.t0);
            center.y = (float)(double)(((float)(double)(((float)(double)(((float)(double)(iso.m4 * source.center.y)) + ((float)(double)(source.center.x * iso.m3))))
                + ((float)(double)(iso.m5 * source.center.z)))) + iso.t1);
            center.z = (float)(double)(((float)(double)(((float)(double)(((float)(double)(iso.m7 * source.center.y)) + ((float)(double)(iso.m6 * source.center.x))))
                + ((float)(double)(iso.m8 * source.center.z)))) + iso.t2);

            halfExtent.x = (float)(double)(((float)(double)(((float)(double)(Math.Abs(iso.m1) * source.halfExtent.y))
                + ((float)(double)(Math.Abs(iso.m0) * source.halfExtent.x))))
                + ((float)(double)(Math.Abs(iso.m2) * source.halfExtent.z)));
            halfExtent.y = (float)(double)(((float)(double)(((float)(double)(Math.Abs(iso.m4) * source.halfExtent.y))
                + ((float)(double)(Math.Abs(iso.m3) * source.halfExtent.x))))
                + ((float)(double)(Math.Abs(iso.m5) * source.halfExtent.z)));
            halfExtent.z = (float)(double)(((float)(double)(((float)(double)(Math.Abs(iso.m7) * source.halfExtent.y))
                + ((float)(double)(Math.Abs(iso.m6) * source.halfExtent.x))))
                + ((float)(double)(Math.Abs(iso.m8) * source.halfExtent.z)));
        }

        public void ToBytes(byte[] b, int o)
        {
            center.ToBytes(b, o);
            halfExtent.ToBytes(b, o + 12);
        }

        public static GmBoxAligned FromBytes(byte[] b, int o)
        {
            GmBoxAligned r = default(GmBoxAligned);
            r.center = GmVec3.FromBytes(b, o);
            r.halfExtent = GmVec3.FromBytes(b, o + 12);
            return r;
        }
    }

    /// <summary>Surface type ids (the polymorphic GmSurf family discriminator).</summary>
    public static class GmSurfTypes
    {
        public const byte GmSurfSphere = 0;
        public const byte GmSurfEllipsoid = 1;
        public const byte GmSurfPlane = 2;
        public const byte GmSurfQuadHeight = 3;
        public const byte GmSurfTriangleHeight = 4;
        public const byte GmSurfPolygon = 5;
        public const byte GmSurfBox = 6;
        public const byte GmSurfMesh = 7;
        public const byte GmSurfCylinder = 8;
    }

    /// <summary>The 8-byte GmSurf base header (vtable word is provenance only).</summary>
    [Serializable]
    public struct GmSurf
    {
        public uint vtable;
        public ushort materialIndex;
        public byte surfType;
        public byte reserved;
    }

    [Serializable]
    public struct GmSurfSphere
    {
        public GmSurf @base;
        public float radius;
    }

    [Serializable]
    public struct GmSurfEllipsoid
    {
        public GmSurf @base;
        public GmVec3 radii;
    }

    [Serializable]
    public struct GmSurfBox
    {
        public GmSurf @base;
        public GmVec3 center;
        public GmVec3 halfExtent;
    }

    /// <summary>0x4c-byte polygon surface (vertices are mesh-local points).</summary>
    [Serializable]
    public struct GmSurfPolygon
    {
        public GmSurf @base;
        public GmVec3 v0;
        public GmVec3 v1;
        public GmVec3 v2;
        public GmVec3 v3;
        public byte vertexCount;
        public byte r0;
        public byte r1;
        public byte r2;
        public GmVec3 normal;
        public uint oneSided;
    }

    /// <summary>0x20-byte mesh face; vertex[3] indexes the owning mesh's vertices.</summary>
    [Serializable]
    public struct GmSurfMeshFace
    {
        public GmVec3 normal;
        public uint reserved0c;
        public uint vertex0;
        public uint vertex1;
        public uint vertex2;
        public ushort materialIndex;
        public ushort reserved1e;

        public uint Vertex(int i)
        {
            switch (i)
            {
                case 0: return vertex0;
                case 1: return vertex1;
                default: return vertex2;
            }
        }
    }

    /// <summary>0x20-byte octree node of a mesh.</summary>
    [Serializable]
    public struct GmSurfMeshNode
    {
        public uint skipCount;
        public GmBoxAligned boxAligned;
        public uint faceIndex;
    }

    /// <summary>
    /// A triangle mesh surface: owned vertex/face/node arrays (class —
    /// assigned by reference, cloned explicitly where the reference clones).
    /// </summary>
    [Serializable]
    public sealed class GmSurfMesh
    {
        public GmSurf @base;
        public uint vertexCount;
        public GmVec3[] vertices;
        public uint faceCount;
        public GmSurfMeshFace[] faces;
        public uint nodeCount;
        public GmSurfMeshNode[] nodes;

        public GmSurfMesh Clone()
        {
            GmSurfMesh m = new GmSurfMesh();
            m.@base = @base;
            m.vertexCount = vertexCount;
            m.vertices = (GmVec3[])vertices.Clone();
            m.faceCount = faceCount;
            m.faces = (GmSurfMeshFace[])faces.Clone();
            m.nodeCount = nodeCount;
            m.nodes = (GmSurfMeshNode[])nodes.Clone();
            return m;
        }
    }

    /// <summary>The polymorphic GmSurf* family behind CPlugSurface::geom.</summary>
    [Serializable]
    public abstract class GmSurfGeom
    {
        public abstract byte SurfType();
        public abstract GmSurf Base();

        public GmSurfSphere AsSphere()
        {
            if (this is GmSurfGeomSphere s) return s.shape;
            throw new InvalidOperationException("tmnf: surface is not a sphere");
        }

        public GmSurfEllipsoid AsEllipsoid()
        {
            if (this is GmSurfGeomEllipsoid e) return e.shape;
            throw new InvalidOperationException("tmnf: surface is not an ellipsoid");
        }

        public GmSurfBox AsBox()
        {
            if (this is GmSurfGeomBox b) return b.shape;
            throw new InvalidOperationException("tmnf: surface is not a box");
        }

        public GmSurfMesh AsMesh()
        {
            if (this is GmSurfGeomMesh m) return m.mesh;
            throw new InvalidOperationException("tmnf: surface is not a mesh");
        }
    }

    public sealed class GmSurfGeomSphere : GmSurfGeom
    {
        public GmSurfSphere shape;
        public override byte SurfType() => GmSurfTypes.GmSurfSphere;
        public override GmSurf Base() => shape.@base;
    }

    public sealed class GmSurfGeomEllipsoid : GmSurfGeom
    {
        public GmSurfEllipsoid shape;
        public override byte SurfType() => GmSurfTypes.GmSurfEllipsoid;
        public override GmSurf Base() => shape.@base;
    }

    public sealed class GmSurfGeomBox : GmSurfGeom
    {
        public GmSurfBox shape;
        public override byte SurfType() => GmSurfTypes.GmSurfBox;
        public override GmSurf Base() => shape.@base;
    }

    public sealed class GmSurfGeomPolygon : GmSurfGeom
    {
        public GmSurfPolygon shape;
        public override byte SurfType() => GmSurfTypes.GmSurfPolygon;
        public override GmSurf Base() => shape.@base;
    }

    public sealed class GmSurfGeomMesh : GmSurfGeom
    {
        public GmSurfMesh mesh;
        public override byte SurfType() => GmSurfTypes.GmSurfMesh;
        public override GmSurf Base() => mesh.@base;
    }

    /// <summary>0x38-byte collision record (the contact produced by narrowphase).</summary>
    [Serializable]
    public struct GmCollision
    {
        public GmVec3 separation;   /* 0x00 */
        public GmVec3 normal;       /* 0x0c */
        public GmVec3 position;     /* 0x18 */
        public ushort material1;    /* 0x24 */
        public ushort material2;    /* 0x26 */
        public uint flags;          /* 0x28 */
        public GmVec3 faceNormal;   /* 0x2c */
    }

    /// <summary>0x4c-byte physical collision record (buffer element).</summary>
    [Serializable]
    public struct SHmsPhysicalCollision
    {
        public uint corpus1;
        public uint tree1;
        public uint corpus2;
        public uint tree2;
        public GmCollision collision;
        public uint material;
    }

    /// <summary>Sphere/mesh edge cache entry (immutable per face).</summary>
    [Serializable]
    public struct TmnfSphereMeshEdge
    {
        public GmVec3 direction;
        public GmVec3 side;
    }

    /// <summary>Per-face cache: SoA vertices + prepared edges (built by the loader).</summary>
    [Serializable]
    public sealed class TmnfSphereFaceEdges
    {
        public float[][] xyz = new float[3][];   // xyz[axis][lane]
        public TmnfSphereMeshEdge[] edges = new TmnfSphereMeshEdge[3];
        public float[] padding = new float[2];

        public TmnfSphereFaceEdges()
        {
            for (int a = 0; a < 3; a++)
            {
                xyz[a] = new float[4];
            }
        }
    }

    /// <summary>A mesh list pool slot: header (word0 = node count) or entry.</summary>
    [Serializable]
    public struct TmnfMeshPoolSlot
    {
        public uint word0;
        public uint word1;

        public uint NodeCount() => word0;

        public uint EntryNode() => word0;
        public uint EntrySkip() => word1;
    }

    /// <summary>One level of the per-mesh query grid.</summary>
    [Serializable]
    public sealed class TmnfMeshGridLevel
    {
        public double[] origin = new double[3];
        public double cellSize;
        public double invCellSize;
        public double maxHalf;
        public uint[] dims = new uint[3];
        public uint cellCount;
        /// <summary>Header slot per cell (index into the owning grid's pool).</summary>
        public uint[] cells;
    }

    /// <summary>Grid level count per mesh grid.</summary>
    public static class MeshGridConstants
    {
        public const int TmnfMeshGridLevels = 2;
    }

    /// <summary>Optional per-query acceleration for a static mesh.</summary>
    [Serializable]
    public sealed class TmnfMeshGrid
    {
        public TmnfMeshGridLevel[] levels = new TmnfMeshGridLevel[MeshGridConstants.TmnfMeshGridLevels];
        public TmnfMeshPoolSlot[] pool;
        /// <summary>Per face, immutable.</summary>
        public TmnfSphereFaceEdges[] sphereEdges;

        public TmnfMeshGrid()
        {
            for (int i = 0; i < MeshGridConstants.TmnfMeshGridLevels; i++)
            {
                levels[i] = new TmnfMeshGridLevel();
            }
        }
    }

    /// <summary>A located surface pair member (the narrowphase input view).</summary>
    public struct LocatedGmSurf
    {
        public GmSurfGeom surf;
        public GmIso4 iso;
        public uint isLocated;
        /// <summary>Optional accel (null = none). Only mesh secondaries use it.</summary>
        public TmnfMeshQueryAccel accel;
    }

    /// <summary>Per-query accel: precomputed inverse iso + mesh grid.</summary>
    public sealed class TmnfMeshQueryAccel
    {
        public GmIso4? inverseIso;
        public TmnfMeshGrid meshGrid;
    }

    /// <summary>Static cell node sentinel words.</summary>
    public static class CellNodeConstants
    {
        public const uint TmnfCellNodeInternal = uint.MaxValue;
        public const uint TmnfCellNodeEmpty = uint.MaxValue - 1;
    }

    /// <summary>32-byte region-restricted static tree node.</summary>
    [Serializable]
    public struct TmnfStaticCellNode
    {
        public GmBoxAligned boxAligned;
        public uint skip;
        public uint entryIndex;
    }

    /// <summary>The static (track-side) grid over region-restricted tree copies.</summary>
    [Serializable]
    public sealed class TmnfStaticGrid
    {
        public double[] origin = new double[3];
        public double cellSize;
        public double invCellSize;
        public double expand;   // query box half-extent budget E
        public double margin;   // float rounding guard delta
        public uint[] dims = new uint[3];
        public uint cellCount;
        public uint[] cellOffsets;
        public uint[] cellCounts;
        public TmnfStaticCellNode[] nodes;
        public TmnfMeshPoolSlot[] meshPool;
        /// <summary>One per track mesh.</summary>
        public TmnfMeshGrid[] meshGrids;
        /// <summary>Per static entry: grid index or null.</summary>
        public int?[] entryMeshGrids;
        /// <summary>One per static entry.</summary>
        public GmIso4[] entryInverseIsos;
    }

    /// <summary>A collision tree node (arena-indexed children; class: owns surface).</summary>
    [Serializable]
    public sealed class CPlugTree
    {
        public uint objectRef;
        public uint flags;
        public GmBoxAligned boxAligned;
        public GmIso4 localIso;
        public CPlugSurface surface; // null = none
        public int[] children = Array.Empty<int>();

        public CPlugTree Clone()
        {
            CPlugTree t = new CPlugTree();
            t.objectRef = objectRef;
            t.flags = flags;
            t.boxAligned = boxAligned;
            t.localIso = localIso;
            t.surface = surface?.Clone();
            t.children = (int[])children.Clone();
            return t;
        }
    }

    /// <summary>Flat arena of the world's dynamic collision trees + merge buffers.</summary>
    [Serializable]
    public sealed class TreeArena
    {
        public System.Collections.Generic.List<CPlugTree> trees = new System.Collections.Generic.List<CPlugTree>();
        public System.Collections.Generic.List<SHmsSphereBufferContact> contactBuffers = new System.Collections.Generic.List<SHmsSphereBufferContact>();

        public int PushTree(CPlugTree tree)
        {
            trees.Add(tree);
            contactBuffers.Add(null);
            return trees.Count - 1;
        }

        /// <summary>Lazily creates and returns the tree's merge buffer.</summary>
        public SHmsSphereBufferContact ContactBufferFor(int tree)
        {
            if (contactBuffers[tree] == null)
            {
                contactBuffers[tree] = new SHmsSphereBufferContact();
            }
            return contactBuffers[tree];
        }
    }

    /// <summary>One flattened entry of the static collision tree.</summary>
    [Serializable]
    public struct HmsStaticCollisionEntry
    {
        public uint skipCount;
        public GmBoxAligned boxAligned;
        public GmIso4 iso;
        public uint treeFlags;
        public int surface; // index into the track's surface array, -1 = none
        public uint treeRef;
        public uint corpusRef;
    }

    /// <summary>A plug surface: geometry + material remap (class: shared by trees).</summary>
    [Serializable]
    public sealed class CPlugSurface
    {
        public GmSurfGeom geom;
        public byte[] materialIds;
        public uint materialCount;

        public CPlugSurface Clone()
        {
            CPlugSurface s = new CPlugSurface();
            if (geom is GmSurfGeomSphere sp) s.geom = new GmSurfGeomSphere { shape = sp.shape };
            else if (geom is GmSurfGeomEllipsoid el) s.geom = new GmSurfGeomEllipsoid { shape = el.shape };
            else if (geom is GmSurfGeomBox bx) s.geom = new GmSurfGeomBox { shape = bx.shape };
            else if (geom is GmSurfGeomPolygon pg) s.geom = new GmSurfGeomPolygon { shape = pg.shape };
            else if (geom is GmSurfGeomMesh ms) s.geom = new GmSurfGeomMesh { mesh = ms.mesh.Clone() };
            s.materialIds = (byte[])materialIds.Clone();
            s.materialCount = materialCount;
            return s;
        }
    }

    /// <summary>Free functions of the collision module.</summary>
    public static class CollisionHelpers
    {
    /// <summary>Prepares one edge of a triangle for the sphere/mesh kernel.</summary>
    public static TmnfSphereMeshEdge TmnfSphereMeshEdgePrepare(ref GmVec3 vertex, ref GmVec3 next, ref GmVec3 normal)
    {
        GmVec3 edge = new GmVec3((float)(double)(next.x - vertex.x), (float)(double)(next.y - vertex.y), (float)(double)(next.z - vertex.z));
        // 0x1.b7cdfcp-34f — the edge-length floor
        float tinySquared = BitConverter.Int32BitsToSingle(unchecked((int)0x2EB7_CDFC));
        float squared = (float)(double)(((float)(double)(((float)(double)(edge.x * edge.x)) + ((float)(double)(edge.y * edge.y)))) + ((float)(double)(edge.z * edge.z)));
        if (tinySquared < squared)
        {
            float length = Fp.Sqrt(squared);
            float inverse = (float)(double)(1.0f / length);
            edge.x = (float)(double)(inverse * edge.x);
            edge.y = (float)(double)(edge.y * inverse);
            edge.z = (float)(double)(inverse * edge.z);
        }
        GmVec3 side = new GmVec3(
            (float)(double)(((float)(double)(edge.y * normal.z)) - ((float)(double)(edge.z * normal.y))),
            (float)(double)(((float)(double)(normal.x * edge.z)) - ((float)(double)(normal.z * edge.x))),
            (float)(double)(((float)(double)(normal.y * edge.x)) - ((float)(double)(edge.y * normal.x))));
        TmnfSphereMeshEdge r = default(TmnfSphereMeshEdge);
        r.direction = edge;
        r.side = side;
        return r;
    }

    /// <summary>The world iso of a tree under a parent iso (flags bit 2 = local).</summary>
    public static GmIso4 TreeWorldIso(CPlugTree tree, ref GmIso4 parent)
    {
        if ((tree.flags & 4u) == 0)
        {
            return parent;
        }
        GmIso4 result = tree.localIso;
        result.Mult(ref parent);
        return result;
    }

    /// <summary>Transforms a direction by an iso's rotation block.</summary>
    public static void TransformDirection(ref GmVec3 vector, ref GmIso4 iso)
    {
        GmVec3 source = vector;
        GmMat3 rot = default(GmMat3);
        rot.m0 = iso.m0; rot.m1 = iso.m1; rot.m2 = iso.m2;
        rot.m3 = iso.m3; rot.m4 = iso.m4; rot.m5 = iso.m5;
        rot.m6 = iso.m6; rot.m7 = iso.m7; rot.m8 = iso.m8;
        vector.SetMultMat3(ref source, ref rot);
    }

    /// <summary>Builds the relative iso of a located pair (first in second's frame).</summary>
    public static GmIso4 RelativeIso(ref LocatedGmSurf first, ref LocatedGmSurf second)
    {
        GmIso4 relative = first.isLocated == 0 ? GmIso4.Identity : first.iso;
        if (second.isLocated != 0)
        {
            if (second.accel != null && second.accel.inverseIso.HasValue)
            {
                GmIso4 inv = second.accel.inverseIso.Value;
                relative.Mult(ref inv);
            }
            else
            {
                GmIso4 inverse = default(GmIso4);
                GmIso4 secondIso = second.iso;
                inverse.SetInverse(ref secondIso);
                relative.Mult(ref inverse);
            }
        }
        return relative;
    }
}

/// <summary>A selected region-restricted node list (pool slice).</summary>
[Serializable]
public struct MeshListNodeList
{
    public uint start;
    public uint count;
}
}
