using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Low)]
public struct MovingAverageJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float2> input; // mergeNative
    [WriteOnly] public NativeArray<float2> output; // smoothNative

    [ReadOnly] public int nw;    // ウィンドウ幅（奇数）
    [ReadOnly] public int half;  // = nw / 2
    [ReadOnly] public int N;     // input.Length

    public void Execute(int i)
    {
        float2 sum = float2.zero;
        int count = 0;

        // i のまわりのウィンドウ
        for (int k = -half; k <= half; k++)
        {
            int idx = math.clamp(i + k, 0, N - 1);

            sum += input[idx];
            count++;
        }

        output[i] = sum / count;
    }
}