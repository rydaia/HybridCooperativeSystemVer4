// Scripts/Bspine/Job/BspineDerivativeJob.cs
// 基底関数行列と制御点からBスプライン曲線上の位置を並列計算するJob

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Low)]
public struct BsplineEvaluateJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float>  Nmat;          // (end-start) × n
    [ReadOnly] public NativeArray<float2> controlPoints;

    [WriteOnly] public NativeArray<float2> C;

    public int n;        // 制御点数
    public int start;    // 元の j start
    // index: 0-Ns
    // j: start-(start+Ns) 
    public void Execute(int index)
    {
        int j = start + index;

        float2 sum = float2.zero;

        int baseIdx = j * n;
        for (int i = 0; i < n; i++)
        {
            sum += Nmat[baseIdx + i] * controlPoints[i];
        }

        C[index] = sum;
    }
}