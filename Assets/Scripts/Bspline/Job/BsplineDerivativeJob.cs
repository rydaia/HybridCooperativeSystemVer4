using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Low)]
public struct BSplineDerivativeJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float2> ctrl; // beta1
    [ReadOnly] public NativeArray<float> knots; // knots

    public int degree; // k-1

    public float u_min_ori;
    public float u_max_ori;

    public float u_min;
    public float u_max;
    
    // 長さ4: [D1, D2, D3, D4]
    [WriteOnly] public NativeArray<float2> results;

    public void Execute(int idx)
    {
        float t = (float)idx / (results.Length - 1); // 0.00001刻み
        // インデックスを元の曲線のuに対応させる
        float u = math.lerp(u_min_ori, u_max_ori, t);

        // 微分曲線のuの範囲内になるようにクランプ
        float u1 = math.clamp(u, u_min, u_max);

        results[idx] = DeBoor(ctrl, knots, u1, degree);
    }

    private float2 DeBoor(NativeArray<float2> c, NativeArray<float> K, float u1, int k)
    {
        // 制御点beta1やbeta2の長さ
        int n = c.Length - 1;
        int span = FindSpan(u1, k, n, K);

        // Burst OK: stackalloc
        // Span<float2> d = stackalloc float2[6]; // 最大5+1くらいあれば十分
        unsafe
        {
            float2* d = stackalloc float2[6]; // 6 = degree最大5の想定

            for (int j = 0; j <= k; j++)
                d[j] = c[span - k + j];

            for (int r = 1; r <= k; r++)
            {
                for (int j = k; j >= r; j--)
                {
                    int idx = span - k + j;
                    // float denom = K[idx + k - r + 1] - K[idx];
                    float denom = K[idx + k - r + 1] - K[idx];
                    float alpha = (denom == 0f) ? 0f : (u1 - K[idx]) / denom;
                    // float alpha = (u1 - K[idx]) / denom;
                    d[j] = math.lerp(d[j - 1], d[j], alpha);
                }
            }
            return d[k];
        }

    }

    private int FindSpan(float u, int degree, int n, NativeArray<float> K)
    {
        int m = K.Length - 1;
        if (u >= K[m - degree]) return n;

        int low = degree;
        int high = n + 1;
        int mid = (low + high) / 2;

        while (u < K[mid] || u >= K[mid + 1])
        {
            if (u < K[mid]) high = mid;
            else low = mid;
            mid = (low + high) / 2;
        }
        return mid;
    }


    // private int FindSpan(float u1, int degree, int n, NativeArray<float> knots)
    // {
    //     if (u1 >= knots[n + 1]) return n;

    //     int low = degree;
    //     int high = n + 1;
    //     int mid = (low + high) / 2;

    //     while (u1 < knots[mid] || u1 >= knots[mid + 1])
    //     {
    //         if (u1 < knots[mid]) high = mid;
    //         else low = mid;
    //         mid = (low + high) / 2;
    //     }
    //     return mid;
    // }
}

[BurstCompile]
public struct DerivativeCtrlJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float2> ctrl;
    [ReadOnly] public NativeArray<float> knots;
    public int degree;       // 今回の次数

    [WriteOnly] public NativeArray<float2> beta;

    public void Execute(int i)
    {
        int p = degree;  
        float denom = knots[i + p + 1] - knots[i + 1];

        float scale = (denom == 0f) ? 0f : (float)p / denom;

        beta[i] = scale * (ctrl[i + 1] - ctrl[i]);
    }
}

[BurstCompile]
public struct EvaluateDerivativeJob : IJob
{
    [ReadOnly] public NativeArray<float2> beta;     // β1,β2,β3,β4
    [ReadOnly] public NativeArray<float> knots;     // knots1～knots4（縮小ノット）
    public int degree;                              // k-1, k-2, k-3, k-4
    public float u;

    public float u_min;
    public float u_max;

    [WriteOnly] public NativeArray<float2> result;

    public void Execute()
    {
        // result[0] = DeBoor(beta, knots, u, degree);

        // float uClamped = math.clamp(u, u_min, u_max);;

        // 有効範囲外なら「0」か「前回値」
        if (u < u_min || u > u_max)
        {
            result[0] = float2.zero;
            return;
        }

        // result[0] = DeBoor(beta, knots, uClamped, degree);
        result[0] = DeBoor(beta, knots, u, degree);
    }

    // -------- DeBoor（縮小ノット対応版） --------
    private float2 DeBoor(NativeArray<float2> c, NativeArray<float> K, float u, int deg)
    {
        int n = c.Length - 1;

        int span = FindSpan(u, deg, n, K);

        unsafe
        {
            float2* d = stackalloc float2[10];

            // ctrl 取り出し
            for (int j = 0; j <= deg; j++)
                d[j] = c[span - deg + j];

            // DeBoor recursion
            for (int r = 1; r <= deg; r++)
            {
                for (int j = deg; j >= r; j--)
                {
                    int idx = span - deg + j;

                    // ★ 縮小ノット列対応（最重要）
                    float denom = K[idx + deg - r + 1] - K[idx];
                    float alpha = (denom == 0f) ? 0f : (u - K[idx]) / denom;

                    d[j] = math.lerp(d[j - 1], d[j], alpha);
                }
            }

            return d[deg];
        }
    }

    private int FindSpan(float u, int degree, int n, NativeArray<float> K)
    {
        // 左端
        if (u <= K[degree])
            return degree;

        // 右端
        if (u >= K[n + 1])
            return n;

        int low  = degree;
        int high = n + 1;
        int mid  = (low + high) >> 1;

        while (u < K[mid] || u >= K[mid + 1])
        {
            if (u < K[mid]) high = mid;
            else            low  = mid;
            mid = (low + high) >> 1;
        }
        return mid;
    }


    // // -------- FindSpan（縮小ノット対応版） --------
    // private int FindSpan(float u, int degree, int n, NativeArray<float> K)
    // {
    //     int m = K.Length - 1;

    //     // 右端境界処理
    //     if (u >= K[m - degree])
    //         return n;

    //     int low  = degree;
    //     int high = n + 1;
    //     int mid  = (low + high) / 2;

    //     while (u < K[mid] || u >= K[mid + 1])
    //     {
    //         if (u < K[mid]) high = mid;
    //         else            low  = mid;
    //         mid = (low + high) / 2;
    //     }
    //     return mid;
    // }
}
