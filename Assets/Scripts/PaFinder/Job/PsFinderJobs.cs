using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

#region Ps1探索

// forループをスレッドで分割して同時に走らせることができる
[BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Low)]
public struct Ps1EvalJob : IJobParallelFor
{
    [ReadOnly] public float2 posCurr;   // 現在位置
    [ReadOnly] public int nBspline;
    [ReadOnly] public int startIdx;   // 評価範囲の開始（ローカル探索用）
    [ReadOnly] public int endIdx;     // 評価範囲の終端（ローカル探索用）
    [ReadOnly] public float epsilon;  // |e·(x - R)| 閾値

    [ReadOnly] public NativeArray<float2> R;   // ベジェ曲線上の点: R[i]=(x,z)
    [ReadOnly] public NativeArray<float2> dR;  // 一階微分 dR/dq を q=i/(n-1) で前計算

    // score[i] は曲線上の i 番目の点 R[i] に対する評価値
    [WriteOnly] public NativeArray<float> score; // 採用→dist^2、不採用→+∞

    public void Execute(int i)
    {
        // 範囲外だけ弾く（端点は許可）
        if (i < 0 || i >= nBspline || i < startIdx || i > endIdx)
        {
            score[i] = float.PositiveInfinity;
            return;
        }

        float2 d = dR[i];
        float len = math.length(d);

        if (len <= 1e-12f) 
        { 
            score[i] = float.PositiveInfinity; 
            return; 
        }

        float2 e = d / len;          // 接線の単位ベクトル
        float2 diff = posCurr - R[i];  // 点との差
        float inner = math.abs(e.x * diff.x + e.y * diff.y);

        if(inner <= epsilon)
        {
            score[i] = math.lengthsq(diff);
        }else
        {
            score[i] = float.PositiveInfinity;
        }
    }
}

#endregion


#region Ps2探索: 半径制約 |R(q1) - R(q)|^2 ≈ L2^2 （全域/範囲）


[BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Low)]
public struct Ps2EvalJob : IJobParallelFor
{
    [ReadOnly] public float2 ps1;     // R(q1)
    [ReadOnly] public float r2;       // L2^2
    [ReadOnly] public int nBspline;
    [ReadOnly] public int startIdx;   // 走査上限（通常 = q1Index）
    [ReadOnly] public int endIdx;     // 走査下限（通常 = 0 or 範囲下限）

    [ReadOnly] public NativeArray<float2> R;

    [WriteOnly] public NativeArray<float> err; // |dist^2 - r2|（小さいほど良い）

    // ループ部分を定義
    public void Execute(int i)
    {
        // qは startIdx から endIdx 方向にのみ有効
        if (i < 0 || i >= nBspline || i > startIdx || i < endIdx)
        {
            err[i] = float.PositiveInfinity;
            return;
        }

        float2 d = ps1 - R[i];
        float d2 = math.lengthsq(d);
        err[i] = math.abs(d2 - r2);
    }
}

#endregion