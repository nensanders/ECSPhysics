﻿using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace PhysicsEngine
{
    [BurstCompile]
    public struct DequeueIntoArray<T> : IJob where T : struct
    {
        public int StartIndex;
        public NativeQueue<T> InputQueue;
        [WriteOnly]
        public NativeArray<T> OutputArray;

        public void Execute()
        {
            int queueCount = InputQueue.Count;
            for (int i = StartIndex; i < StartIndex + queueCount; i++)
            {
                OutputArray[i] = InputQueue.Dequeue();
            }
        }
    }

    public static class PhysicsMath
    {
        public static float3 ProjectOnPlane(float3 vector, float3 onPlane)
        {
            float3 orthogonalComponent = onPlane * math.dot(vector, onPlane);
            return vector - orthogonalComponent;
        }

        public static float3 ProjectOnNormal(float3 vector, float3 onNormal)
        {
            return onNormal * math.dot(vector, onNormal);
        }

        public static uint ExpandBits(uint v)
        {
            v = (v * 0x00010001u) & 0xFF0000FFu;
            v = (v * 0x00000101u) & 0x0F00F00Fu;
            v = (v * 0x00000011u) & 0xC30C30C3u;
            v = (v * 0x00000005u) & 0x49249249u;
            return v;
        }

        public static int CalculateMortonCode(float3 vector)
        {
            vector.x = math.min(math.max(vector.x * 1024.0f, 0.0f), 1023.0f);
            vector.y = math.min(math.max(vector.y * 1024.0f, 0.0f), 1023.0f);
            vector.z = math.min(math.max(vector.z * 1024.0f, 0.0f), 1023.0f);
            uint xx = ExpandBits((uint)vector.x);
            uint yy = ExpandBits((uint)vector.y);
            uint zz = ExpandBits((uint)vector.z);
            return (int)(xx * 4 + yy * 2 + zz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool AABBToAABBOverlap(AABB a, AABB b)
        {
            return (a.Min.x <= b.Max.x && a.Max.x >= b.Min.x) &&
                    (a.Min.z <= b.Max.z && a.Max.z >= b.Min.z) &&
                    (a.Min.y <= b.Max.y && a.Max.y >= b.Min.y);
        }

        public static AABB GetEncompassingAABB(AABB a, AABB b)
        {
            AABB returnAABB = new AABB();

            returnAABB.Min = math.min(a.Min, b.Min);
            returnAABB.Max = math.max(a.Max, b.Max);

            return returnAABB;
        }

        public static void GrowAABB(ref AABB sourceAABB, float3 includePoint)
        {
            sourceAABB.Min.x = math.min(sourceAABB.Min.x, includePoint.x);
            sourceAABB.Min.y = math.min(sourceAABB.Min.y, includePoint.y);
            sourceAABB.Min.z = math.min(sourceAABB.Min.z, includePoint.z);
            sourceAABB.Max.x = math.max(sourceAABB.Max.x, includePoint.x);
            sourceAABB.Max.y = math.max(sourceAABB.Max.y, includePoint.y);
            sourceAABB.Max.z = math.max(sourceAABB.Max.z, includePoint.z);
        }

        public static float3 GetAABBCenter(AABB aabb)
        {
            return (aabb.Min + aabb.Max) * 0.5f;
        }

        public static int GetNextHighestPowerOf2(int v)
        {
            v--;
            v |= v >> 1;
            v |= v >> 2;
            v |= v >> 4;
            v |= v >> 8;
            v |= v >> 16;
            v++;

            return v;
        }

        public static int GetNextLowestPowerOf2(int v)
        {
            v |= (v >> 1);
            v |= (v >> 2);
            v |= (v >> 4);
            v |= (v >> 8);
            v |= (v >> 16);
            return v - (v >> 1);
        }

        public static int GetFirstChildBVHNodeIndex(int index)
        {
            return (index * 2) + 1;
        }

        public static int GetParentBVHNodeIndex(int index)
        {
            if (math.fmod(index, 2) == 0)
            {
                return (index / 2) - 1;
            }
            else
            {
                return index / 2;
            }
        }

        public static int GetIndexForRowAndColumn(int row, int column, int columnCount)
        {
            return column + (row * columnCount);
        }
    }
}