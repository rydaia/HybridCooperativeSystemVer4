using System.Collections.Generic;
using Unity.Mathematics;
using Unity.Collections;
using UnityEngine;
using Unity.Jobs;


public class ControlPointResampler
{
    private float ds;         // 再サンプリング間隔
    private int Nresample;    // 再サンプリング後の点数

    private NativeArray<float> S;          // 累積距離
    private NativeArray<float2> resampled; // 出力

    public void Initialize(float smax_past)
    {
        // float ds = 0.01f;
        this.ds = 0.01f;
        // float totalS = smax_past + smax_future;
        float totalS = smax_past;

        Nresample = Mathf.FloorToInt(totalS / ds) + 1;

        resampled = new NativeArray<float2>(Nresample, Allocator.Persistent);

        Debug.Log($"InitializeResample complete. Length={resampled.Length}");
    }

    public NativeArray<float2> ResampleByDistance(NativeArray<float2> input)
    {
        int n = input.Length;

        // 累積距離 S を作る 
        S = new NativeArray<float>(n, Allocator.Persistent);
        S[0] = 0f;

        for (int i = 1; i < n; i++)
        {
            float d = math.distance(input[i], input[i - 1]);
            S[i] = S[i - 1] + d;
        }

        float Smax = S[n - 1];

        //ds 間隔で targetS を走査
        int idx = 1;  // S[idx-1] < targetS <= S[idx]

        for (int i = 0; i < Nresample; i++)
        {
            float targetS = i * ds;

            if (targetS > Smax) targetS = Smax;

            // S[idx-1] <= targetS <= S[idx] となる idx を探す
            while (idx < n - 1 && S[idx] < targetS)
                idx++;

            float s0 = S[idx - 1];
            float s1 = S[idx];

            float t = (targetS - s0) / (s1 - s0);
            float2 p0 = input[idx - 1];
            float2 p1 = input[idx];

            resampled[i] = math.lerp(p0, p1, t);
        }

        S.Dispose();
        return resampled;
    }

    public void Dispose()
    {
        if (resampled.IsCreated) resampled.Dispose();
    }

    public int GetN()
    {
        return resampled.Length;
    }

    public float2[] GetResampledPointsManaged(NativeArray<float2> resampled)
    {
        int n = resampled.Length;

        float2[] arr = new float2[n];
        for (int i = 0; i < n; i++)
            arr[i] = resampled[i];
        return arr;
    }
}
