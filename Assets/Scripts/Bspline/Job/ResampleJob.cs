// using Unity.Burst;
// using Unity.Collections;
// using Unity.Jobs;
// using Unity.Mathematics;

// [BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Low)]
// public struct BuildCumulativeDistanceJob : IJob
// {
//     [ReadOnly] public NativeArray<float2> input;
//     public NativeArray<float> S;

//     public void Execute()
//     {
//         int n = input.Length;
//         S[0] = 0f;

//         for (int i = 1; i < n; i++)
//         {
//             float d = math.distance(input[i], input[i - 1]);
//             S[i] = S[i - 1] + d;
//         }
//     }
// }

// [BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Low)]
// public struct ResampleJob : IJobParallelFor
// {
//     [ReadOnly] public NativeArray<float2> input;
//     [ReadOnly] public NativeArray<float> S;

//     public float ds;
//     public float Smax;
//     public NativeArray<float2> resampled;

//     public void Execute(int i)
//     {
//         float targetS = i * ds;
//         if (targetS > Smax) targetS = Smax;

//         // idx を探す（逐次だが入力サイズ小なら許容）
//         int n = S.Length;
//         int idx = 1;
//         while (idx < n - 1 && S[idx] < targetS)
//             idx++;

//         float s0 = S[idx - 1];
//         float s1 = S[idx];

//         if (math.abs(s1 - s0) < 1e-6f)
//         {
//             resampled[i] = input[idx];
//             return;
//         }


//         float t = (targetS - s0) / (s1 - s0);

//         float2 p0 = input[idx - 1];
//         float2 p1 = input[idx];

//         resampled[i] = math.lerp(p0, p1, t);
//     }
// }