using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Low)]
public struct FutureJob : IJobParallelFor
{
    [ReadOnly] public float2 Tp;
    [ReadOnly] public float theta;
    [ReadOnly] public float kappa;
    [ReadOnly] public float d;

    [WriteOnly] public NativeArray<float2> output; // nf 個ぶん

    public void Execute(int i)
    {
        // float theta_k = theta + kappa * ds * i;
        float theta_k = theta + kappa * d * i;
        float s = d * (i + 1);

        float x = Tp.x + math.cos(theta_k) * s;
        float y = Tp.y + math.sin(theta_k) * s;

        output[i] = new float2(x, y);
    }
}

[BurstCompile]
public struct MergeJob : IJobParallelFor
{

    [ReadOnly] public NativeArray<float2> past;
    [ReadOnly] public int pastHead;
    [ReadOnly] public int pastCount;
    [ReadOnly] public int np;

    [ReadOnly] public NativeArray<float2> future;
    [WriteOnly] public NativeArray<float2> merged;

    public void Execute(int i)
    {
        if (i < np)
        {
            if (i < pastCount)
            {
                int idx = (pastHead + i) % np;
                merged[i] = past[idx];
            }
        }
        else
        {
            int f = i - np;
            merged[i] = future[f];
        }
    }
}