using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

public class ControlPointSmoother
{

    NativeArray<float2> smoothedNative; // np + nf

    int Nsm;
    int nw = 5;

    // スムージング点列データの初期化
    // ここではjobは使わない
    public void Initialize(int totalPoints, CalculationManager calc)
    {
        // 1001 - (5 - 1) / 2 = 999
        Nsm = totalPoints - (nw - 1) / 2;

        smoothedNative = new NativeArray<float2>(Nsm, Allocator.Persistent);
        smoothedNative = ApplyMovingAverage(calc.cpQueue.GetMergedNative());
        Debug.Log($"InitializeSmoothData complete. Length={smoothedNative.Length}");
    }

    public NativeArray<float2> ApplyMovingAverage(NativeArray<float2> input)
    {

        int half = nw / 2;

        var job = new MovingAverageJob
        {
            input = input,
            output = smoothedNative,
            nw = nw,
            half = half,
            N = input.Length
        };

        // ここでJobの実行
        JobHandle handle = job.Schedule(Nsm, 64);
        handle.Complete();

        return smoothedNative;
    }

    public NativeArray<float2> GetSmoothedNative()
    {
        return smoothedNative;
    }

    public void Dispose()
    {
        if (smoothedNative.IsCreated)
            smoothedNative.Dispose();
    }

    public float2[] GetSmoothedPointsManaged()
    {
        float2[] arr = new float2[Nsm];
        for (int i = 0; i < Nsm; i++)
            arr[i] = smoothedNative[i];
        return arr;
    }

    public int GetNsm()
    {
        return Nsm;
    }
}
