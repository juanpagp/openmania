using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// The comparison the response uses to sort physical collisions
    /// (0x00547C80). Per field: if !(b &lt;= a) return 1; if b &lt; a return -1;
    /// else continue. NaN fails <c>b &lt;= a</c> and returns 1, exactly like the
    /// original.
    /// </summary>
    public static class BufferCompare
    {
        private static int Ord(float b, float a)
        {
            if (!(b <= a)) return 1;
            if (b < a) return -1;
            return 0;
        }

        public static int ShmsPhysicalCollisionCompare(ref SHmsPhysicalCollision a, ref SHmsPhysicalCollision b)
        {
            int o = Ord(b.collision.position.x, a.collision.position.x);
            if (o != 0) return o;
            o = Ord(b.collision.position.y, a.collision.position.y);
            if (o != 0) return o;
            o = Ord(b.collision.position.z, a.collision.position.z);
            if (o != 0) return o;
            o = Ord(b.collision.normal.x, a.collision.normal.x);
            if (o != 0) return o;
            o = Ord(b.collision.normal.y, a.collision.normal.y);
            if (o != 0) return o;
            o = Ord(b.collision.normal.z, a.collision.normal.z);
            if (o != 0) return o;
            o = Ord(b.collision.separation.x, a.collision.separation.x);
            if (o != 0) return o;
            o = Ord(b.collision.separation.y, a.collision.separation.y);
            if (o != 0) return o;
            o = Ord(b.collision.separation.z, a.collision.separation.z);
            if (o != 0) return o;

            if (a.collision.flags == 0 && b.collision.flags != 0)
            {
                return -1;
            }
            return 1;
        }
    }

    /// <summary>
    /// The game's collision record buffer (a growable list of 0x4c-byte
    /// records). <see cref="At"/>/<see cref="AtMut"/> mirror the reference's
    /// indexed access; <see cref="AddNewElem"/> appends.
    /// </summary>
    [Serializable]
    public sealed class CFastBufferShmsPhysicalCollision
    {
        public SHmsPhysicalCollision[] data = new SHmsPhysicalCollision[0];
        public int count;

        public ref SHmsPhysicalCollision At(int index) => ref data[index];

        public uint Count() => (uint)count;

        /// <summary>Appends one default record and returns its index.</summary>
        public int AddNewElem()
        {
            if (count == data.Length)
            {
                int newLen = data.Length == 0 ? 0x32 : data.Length * 2;
                Array.Resize(ref data, newLen);
            }
            data[count] = default(SHmsPhysicalCollision);
            return count++;
        }

        public void Clear() => count = 0;

        public void Push(SHmsPhysicalCollision record)
        {
            if (count == data.Length)
            {
                int newLen = data.Length == 0 ? 0x32 : data.Length * 2;
                Array.Resize(ref data, newLen);
            }
            data[count++] = record;
        }

        /// <summary>
        /// 0x00547DE0 — the game's MSVC quicksort, transcribed. Indices stand
        /// in for the C's byte pointers; the algorithm, pivots, swaps and
        /// iteration order are identical, so the resulting permutation
        /// (including equal-element order) is identical. UNSTABLE by design.
        /// </summary>
        public void Qsort(BufferCompareFn compare)
        {
            if (count < 2)
            {
                return;
            }

            int[] loStack = new int[30];
            int[] hiStack = new int[30];
            int stackTop = 0;

            int lo = 0;
            int hi = count - 1;

            while (true)
            {
                uint count_ = (uint)(hi - lo + 1);
                while (count_ > 8)
                {
                    int mid = lo + (int)(count_ >> 1);

                    if (compare(ref data[lo], ref data[mid]) > 0)
                    {
                        Swap(lo, mid);
                    }
                    if (compare(ref data[lo], ref data[hi]) > 0)
                    {
                        Swap(lo, hi);
                    }
                    if (compare(ref data[mid], ref data[hi]) > 0)
                    {
                        Swap(mid, hi);
                    }

                    int left = lo;
                    int right = hi;
                    int mid2 = mid;
                    while (true)
                    {
                        if (left < mid2)
                        {
                            while (true)
                            {
                                left += 1;
                                if (mid2 <= left)
                                {
                                    break;
                                }
                                if (compare(ref data[left], ref data[mid2]) >= 1)
                                {
                                    break;
                                }
                            }
                        }
                        if (mid2 <= left)
                        {
                            while (true)
                            {
                                left += 1;
                                if (hi < left)
                                {
                                    break;
                                }
                                if (compare(ref data[left], ref data[mid2]) >= 1)
                                {
                                    break;
                                }
                            }
                        }

                        while (true)
                        {
                            right -= 1;
                            if (right <= mid2)
                            {
                                break;
                            }
                            if (compare(ref data[right], ref data[mid2]) <= 0)
                            {
                                break;
                            }
                        }

                        if (left > right)
                        {
                            break;
                        }
                        Swap(left, right);
                        if (mid2 == right)
                        {
                            mid2 = left;
                        }
                    }

                    right += 1;
                    if (mid2 < right)
                    {
                        while (true)
                        {
                            right -= 1;
                            if (right <= mid2)
                            {
                                break;
                            }
                            if (compare(ref data[right], ref data[mid2]) != 0)
                            {
                                break;
                            }
                        }
                        if (mid2 < right)
                        {
                            // goto partitioned
                            if (right - lo < hi - left)
                            {
                                if (left < hi)
                                {
                                    loStack[stackTop] = left;
                                    hiStack[stackTop] = hi;
                                    stackTop += 1;
                                }
                                hi = right;
                                if (right <= lo)
                                {
                                    break;
                                }
                            }
                            else
                            {
                                if (lo < right)
                                {
                                    loStack[stackTop] = lo;
                                    hiStack[stackTop] = right;
                                    stackTop += 1;
                                }
                                lo = left;
                                if (hi <= left)
                                {
                                    break;
                                }
                            }
                            count_ = (uint)(hi - lo + 1);
                            continue;
                        }
                    }
                    if (mid2 <= right)
                    {
                        while (true)
                        {
                            right -= 1;
                            if (right <= lo)
                            {
                                break;
                            }
                            if (compare(ref data[right], ref data[mid2]) != 0)
                            {
                                break;
                            }
                        }
                    }

                    // partitioned:
                    if (right - lo < hi - left)
                    {
                        if (left < hi)
                        {
                            loStack[stackTop] = left;
                            hiStack[stackTop] = hi;
                            stackTop += 1;
                        }
                        hi = right;
                        if (right <= lo)
                        {
                            break;
                        }
                    }
                    else
                    {
                        if (lo < right)
                        {
                            loStack[stackTop] = lo;
                            hiStack[stackTop] = right;
                            stackTop += 1;
                        }
                        lo = left;
                        if (hi <= left)
                        {
                            break;
                        }
                    }
                    count_ = (uint)(hi - lo + 1);
                }

                if (lo < hi)
                {
                    // shortsort: selection sort, transcribed
                    int hiS = hi;
                    while (lo < hiS)
                    {
                        int max = lo;
                        for (int scan = lo + 1; scan <= hiS; scan++)
                        {
                            if (compare(ref data[scan], ref data[max]) > 0)
                            {
                                max = scan;
                            }
                        }
                        Swap(max, hiS);
                        hiS -= 1;
                    }
                }
                stackTop -= 1;
                if (stackTop < 0)
                {
                    return;
                }
                lo = loStack[stackTop];
                hi = hiStack[stackTop];
            }
        }

        private void Swap(int a, int b)
        {
            SHmsPhysicalCollision t = data[a];
            data[a] = data[b];
            data[b] = t;
        }

        public delegate int BufferCompareFn(ref SHmsPhysicalCollision a, ref SHmsPhysicalCollision b);
    }

    /// <summary>CHmsCollisionBuffer: a record buffer with the game's capacity of 50.</summary>
    [Serializable]
    public sealed class CHmsCollisionBuffer
    {
        public CFastBufferShmsPhysicalCollision collisions = new CFastBufferShmsPhysicalCollision();

        /// <summary>Appends and returns a mutable reference to the new collision.</summary>
        public ref GmCollision AddCollision()
        {
            int idx = collisions.AddNewElem();
            return ref collisions.data[idx].collision;
        }

        public ref GmCollision GetCollision(int index)
        {
            return ref collisions.data[index].collision;
        }

        public ref SHmsPhysicalCollision AtMut(int index)
        {
            return ref collisions.data[index];
        }

        public uint GetCount()
        {
            return collisions.Count();
        }
    }

    /// <summary>SHmsSphereBufferContact: a mergeable per-sphere buffer.</summary>
    [Serializable]
    public sealed class SHmsSphereBufferContact
    {
        public CHmsCollisionBuffer @base = new CHmsCollisionBuffer();
        public uint active;
    }
}
