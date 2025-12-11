using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Low)]
public struct BSplineJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float2> ctrl;  // 制御点
    [ReadOnly] public NativeArray<float> knots;  // ノット

    [WriteOnly] public NativeArray<float2> output; // 出力点（sampleCount 個）

    public int k;   // 次数
    public int n;   // 制御点数−1
    public int m;
    public float u_min;
    public float u_max;

    public void Execute(int idx)
    {

        // t を 0→1 に動かすと
        // u は 0→997 を動く

        // 整数インデックスを少数に変換
        float t = (float)idx / (output.Length - 1); // 0.00001刻み
        float u = math.lerp(u_min, u_max, t); // 

        // output[idx] = DeBoor(u);
        output[idx] = DeBoor(u, ctrl, k);
    }

    // k=5の時
    //     d0 = ctrl[span - 5];
    //     d1 = ctrl[span - 4];
    //     d2 = ctrl[span - 3];
    //     d3 = ctrl[span - 2];
    //     d4 = ctrl[span - 1];
    //     d5 = ctrl[span];

    // 1段目
    //     d0 = math.lerp(d0, d1, Alpha(u, span - 5, 5));
    //     d1 = math.lerp(d1, d2, Alpha(u, span - 4, 5));
    //     d2 = math.lerp(d2, d3, Alpha(u, span - 3, 5));
    //     d3 = math.lerp(d3, d4, Alpha(u, span - 2, 5));
    //     d4 = math.lerp(d4, d5, Alpha(u, span - 1, 5));

    // 2段目
    //     d0 = math.lerp(d0, d1, Alpha(u, span - 4, 4));
    //     d1 = math.lerp(d1, d2, Alpha(u, span - 3, 4));
    //     d2 = math.lerp(d2, d3, Alpha(u, span - 2, 4));
    //     d3 = math.lerp(d3, d4, Alpha(u, span - 1, 4));


    // 3段目
    //     d0 = math.lerp(d0, d1, Alpha(u, span - 3, 3));
    //     d1 = math.lerp(d1, d2, Alpha(u, span - 2, 3));
    //     d2 = math.lerp(d2, d3, Alpha(u, span - 1, 3));

    // 4段目
    //     d0 = math.lerp(d0, d1, Alpha(u, span - 2, 4));
    //     d1 = math.lerp(d1, d2, Alpha(u, span - 1, 4));


    // 5段目
    //     d0 = math.lerp(d0, d1, Alpha(u, span - 5, 1));

    private float2 DeBoor(float u, NativeArray<float2> P, int degree)
    {
        int span = FindSpan(u);

        unsafe
        {
            float2* d = stackalloc float2[6]; // degree max = 5 で固定確保

            // 初期化
            for (int j = 0; j <= degree; j++)
            {
                int idx = span - degree + j;
                d[j] = P[math.clamp(idx, 0, P.Length - 1)];
            }

            // De Boor iteration
            for (int r = 1; r <= degree; r++)
            {
                for (int j = degree; j >= r; j--)
                {
                    int i = span - degree + j;
                    // float denom = knots[i + degree - r + 1] - knots[i];
                    // float alpha = (denom == 0) ? 0 : (u - knots[i]) / denom;

                    float alpha = Alpha(u, i, r, degree);

                    d[j] = math.lerp(d[j - 1], d[j], alpha);
                }
            }

            return d[degree];
        }
    }

    private float Alpha(float u, int i, int r, int degree)
    {
        float denom = knots[i + degree - r + 1] - knots[i];
        return (denom == 0) ? 0 : (u - knots[i]) / denom;
    }

    //　二分探索 binary search
    private int FindSpan(float u)
    {
        if (u >= knots[n + 1])
            return n;

        int low = k;
        int high = n + 1;
        int mid = (low + high) / 2;

        while (u < knots[mid] || u >= knots[mid + 1])
        {
            if (u < knots[mid]) high = mid;
            else low = mid;
            mid = (low + high) / 2;
        }

        return mid;
    }
}
