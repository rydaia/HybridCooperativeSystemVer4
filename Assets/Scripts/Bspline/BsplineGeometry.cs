using Unity.Mathematics;
using UnityEngine;
using System.IO;
using System.Text;
using System.Globalization;
using System;
using Unity.Profiling;
using Unity.Jobs;
using Unity.Collections;


public class BsplineGeometry
{
    static ProfilerMarker bsplineGeometryMarker =
        new ProfilerMarker("BsplineGeometry.CalculationBsplineGeometry()");
    static ProfilerMarker GeneratePointsMarker = new ProfilerMarker("BSplineCurve.generatePoints");
    static ProfilerMarker DerivativePointsMarker = new ProfilerMarker("BSplineCurve.DerivativeAllPoints()");


    private PsFinder psFinder;

    //　制御点データ
    NativeArray<float2> ctrl;
    //　制御点データ　 1階微分
    NativeArray<float2> beta1;
    //　制御点データ 1階微分
    NativeArray<float2> beta2;
    //　制御点データ 1階微分
    NativeArray<float2> beta3;
    //　制御点データ 1階微分
    NativeArray<float2> beta4;
    // ノットベクトル
    NativeArray<float> knots;
    NativeArray<float> knots1, knots2, knots3, knots4;
    //　Bスプライン曲線 R(u)
    NativeArray<float2> points;   // 出力点　N
    // 1階微分　dR(u)/du
    NativeArray<float2> derivative1;   // 出力点　N

    public TrajectoryGenerator trajectoryGenerator;


    int Nctrl; // 制御点の総数　1202
    int N;// 120100
    public float ds;
    private int k;   // 次数（3）
    private int n;   // 制御点数−1
    int m;   // ノット数　可変
    public float[] u;
    private float u_min;
    private float u_max;

    private float rx1, ry1;
    private float d1Rx1du11, d1Ry1du11;
    private float d2Rx1du12, d2Ry1du12;
    private float d3Rx1du13, d3Ry1du13;
    private float d4Rx1du14, d4Ry1du14;

    private float rx2, ry2;
    private float d1Rx2du21, d1Ry2du21;
    private float d2Rx2du22, d2Ry2du22;
    private float d3Rx2du23, d3Ry2du23;
    private float d4Rx2du24, d4Ry2du24;

    public BsplineGeometry()
    {
    }

    public void Initialize(TrajectoryGenerator trajectoryGenerator, PsFinder finder)
    {
        this.trajectoryGenerator = trajectoryGenerator;
        this.psFinder = finder;

        ctrl = new NativeArray<float2>(trajectoryGenerator.GetNctrl(), Allocator.Persistent);
        // ② Nctrl を使って n, m を決定
        k = trajectoryGenerator.GetK();               
        n = trajectoryGenerator.GetNctrl() - 1;      // ← 1201
        m = n + k + 1; // 1001
        // ⑤ 出力配列を確保
        N = trajectoryGenerator.GetN();

        points = new NativeArray<float2>(N, Allocator.Persistent);
        derivative1 = new NativeArray<float2>(N, Allocator.Persistent);

        knots = new NativeArray<float>(m + 1, Allocator.Persistent);
        knots = GenerateKnots(knots, k, n ,m);
        knots1 = SubKnots(knots, 1);
        knots2 = SubKnots(knots, 2);
        knots3 = SubKnots(knots, 3);
        knots4 = SubKnots(knots, 4);

        beta1 = new NativeArray<float2>(n, Allocator.Persistent);
        beta2 = new NativeArray<float2>(n - 1, Allocator.Persistent);
        beta3 = new NativeArray<float2>(n - 2, Allocator.Persistent);
        beta4 = new NativeArray<float2>(n - 3, Allocator.Persistent);

        ds = trajectoryGenerator.GetDs();

        u_min = knots[k];
        u_max = knots[n + 1];

        Debug.Log("BsplineGeometry Initialize 完了");
    }


    public void CalculationBsplineGeometry()
    {

        using (bsplineGeometryMarker.Auto())
        {


            // Ps探索で見つけたインデックスを取得
            // int idx = PsFinder.u1Index;
            int u1Index = psFinder.GetU1Index();
            int u2Index = psFinder.GetU2Index();

            // deboorで使用する制御点を再度作成 beta1, beta2, beta3, beta4
            // CalculateDerivativeControlPointsJob();

            SetD1Rx1du11(trajectoryGenerator.bsplineDerivativeNative[u1Index].x);
            SetD1Ry1du11(trajectoryGenerator.bsplineDerivativeNative[u1Index].y);

            // R(u2)
            SetRx2(trajectoryGenerator.bsplineNative[u2Index].x);
            SetRy2(trajectoryGenerator.bsplineNative[u2Index].y);

            SetD1Rx2du21(trajectoryGenerator.bsplineDerivativeNative[u2Index].x);
            SetD1Ry2du21(trajectoryGenerator.bsplineDerivativeNative[u2Index].y);

            // R(u)の各u微分を計算
            EvaluateDerivativesAtU(u1Index, u2Index);
        }
    }

    NativeArray<float> SubKnots(NativeArray<float> knots, int r)
    {
        int newLen = knots.Length - 2 * r;
        var newKnots = new NativeArray<float>(newLen, Allocator.Persistent);

        for (int i = 0; i < newLen; i++)
            newKnots[i] = knots[i + r];

        return newKnots;
    }

    private NativeArray<float> GenerateKnots(NativeArray<float> K, int k, int n, int m)
    {
        // 開一様ノット
        for (int i = 0; i <= k; i++)
            K[i] = 0;

        for (int i = k + 1; i <= n; i++)
            K[i] = i - k;

        for (int i = n + 1; i < m + 1; i++)
            K[i] = n - k + 1; // 996

        // Debug.Log($"K:{K[0]}, {K[1]}, {K[2]}, {K[3]}, {K[4]}, ... , {K[m-5]}, {K[m-4]}, {K[m-3]}, {K[m-2]}, {K[m-1]}");

        return K;
    }

        // 制御点のサイズが可変になるため
    public NativeArray<float2> Approximate(NativeArray<float2> input)
    {
        int Nctrl = input.Length;

        n = Nctrl - 1;
        m = n + k + 1;

        ctrl.CopyFrom(input);

        u_min = knots[k]; // u_min = knots[3] = 0
        u_max = knots[n + 1]; // u_max = knots[1001] = 995

        // points
        using(GeneratePointsMarker.Auto())
        {
            var job = new BSplineJob {
                ctrl = ctrl,
                knots = knots,
                output = points,
                k = k,
                n = n, 
                m = m,
                u_min = u_min, 
                u_max = u_max
            };

            JobHandle handle = job.Schedule(N, 64);
            handle.Complete();
        }

        return points;
    }

    public NativeArray<float2> DerivativeAllPoints()
    {

        CalculateDerivativeControlPointsJob();

        // Debug.Log($"u_min ={u_min}, u_max={u_max}");
        // Debug.Log($"beta1[0]={geo.GetBeta1()[0]}, beta1[degree]={geo.GetBeta1()[k-1]}");
        // Debug.Log($"knots1[degree]={geo.GetKnots1()[k-1]}, knots1[n+1]={geo.GetKnots1()[n+1]}");

        using(DerivativePointsMarker.Auto())
        {
            var job = new BSplineDerivative1Job {
                ctrl = beta1,
                knots = knots1,
                degree = k-1,
                u_min = u_min, // 0
                u_max = u_max, // 996
                results = derivative1
            };

            JobHandle handle = job.Schedule(N, 64);
            handle.Complete();
        }

        return derivative1;
    }


    private float Alpha(float u, int i, int r)
    {
        float denom = knots[i + k - r + 1] - knots[i];
        return (denom == 0) ? 0 : (u - knots[i]) / denom;
    }

    public float GetUMin()
    {
        return u_min;
    }

    public float GetUMax()
    {
        return u_max;
    }

    public void CalculateDerivativeControlPointsJob()
    {
        // 制御点を取得
        this.ctrl = trajectoryGenerator.resampledNative;

        var job1 = new DerivativeCtrlJob {
            ctrl = ctrl,
            knots = knots,
            degree = k,       // ← trajectory で使っている本来の次数
            beta = beta1
        }.Schedule(beta1.Length, 64);

        var job2 = new DerivativeCtrlJob {
            ctrl = beta1,
            knots = knots1,
            degree = k - 1,
            beta = beta2
        }.Schedule(beta2.Length, 64, job1);

        var job3 = new DerivativeCtrlJob {
            ctrl = beta2,
            knots = knots2,
            degree = k - 2,
            beta = beta3
        }.Schedule(beta3.Length, 64, job2);

        var job4 = new DerivativeCtrlJob {
            ctrl = beta3,
            knots = knots3,
            degree = k - 3,
            beta = beta4
        }.Schedule(beta4.Length, 64, job3);

        job4.Complete();
        // Debug.Log($"beta1[0]={beta1[0]}, beta1[1000]={beta1[1000]}");

    }

    public void EvaluateDerivativesAtU(int idx1, int idx2)
    {
        float u1 = math.lerp(u_min, u_max, (float)idx1 / (N - 1));
        float u2 = math.lerp(u_min, u_max, (float)idx2 / (N - 1));

        // 結果バッファ（1〜4階微分）
        var d1u1 = new NativeArray<float2>(1, Allocator.TempJob);
        var d2u1 = new NativeArray<float2>(1, Allocator.TempJob);
        var d3u1 = new NativeArray<float2>(1, Allocator.TempJob);
        var d4u1 = new NativeArray<float2>(1, Allocator.TempJob);

        var d1u2 = new NativeArray<float2>(1, Allocator.TempJob);
        var d2u2 = new NativeArray<float2>(1, Allocator.TempJob);
        var d3u2 = new NativeArray<float2>(1, Allocator.TempJob);
        var d4u2 = new NativeArray<float2>(1, Allocator.TempJob);

        // --------------------
        // u1 の微分計算
        // --------------------
        var j1 = new EvaluateDerivativeJob {
            beta = beta1,
            knots = knots1,
            degree = k - 1,
            u = u1,
            result = d1u1
        }.Schedule();

        var j2 = new EvaluateDerivativeJob {
            beta = beta2,
            knots = knots2,
            degree = k - 2,
            u = u1,
            result = d2u1
        }.Schedule(j1);

        var j3 = new EvaluateDerivativeJob {
            beta = beta3,
            knots = knots3,
            degree = k - 3,
            u = u1,
            result = d3u1
        }.Schedule(j2);

        var j4 = new EvaluateDerivativeJob {
            beta = beta4,
            knots = knots4,
            degree = k - 4,
            u = u1,
            result = d4u1
        }.Schedule(j3);


        // --------------------
        // u2 の微分計算（u1 完了後）
        // --------------------
        var j5 = new EvaluateDerivativeJob {
            beta = beta1,
            knots = knots1,
            degree = k - 1,
            u = u2,
            result = d1u2
        }.Schedule(j4);

        var j6 = new EvaluateDerivativeJob {
            beta = beta2,
            knots = knots2,
            degree = k - 2,
            u = u2,
            result = d2u2
        }.Schedule(j5);

        var j7 = new EvaluateDerivativeJob {
            beta = beta3,
            knots = knots3,
            degree = k - 3,
            u = u2,
            result = d3u2
        }.Schedule(j6);

        var j8 = new EvaluateDerivativeJob {
            beta = beta4,
            knots = knots4,
            degree = k - 4,
            u = u2,
            result = d4u2
        }.Schedule(j7);

        j8.Complete();

        // --- U1 の微分結果反映 ---
        SetD1Rx1du11(d1u1[0].x);
        SetD1Ry1du11(d1u1[0].y);
        SetD2Rx1du12(d2u1[0].x);
        SetD2Ry1du12(d2u1[0].y);
        SetD3Rx1du13(d3u1[0].x);
        SetD3Ry1du13(d3u1[0].y);
        SetD4Rx1du14(d4u1[0].x);
        SetD4Ry1du14(d4u1[0].y);

        // --- U2 ---
        SetD1Rx2du21(d1u2[0].x);
        SetD1Ry2du21(d1u2[0].y);
        SetD2Rx2du22(d2u2[0].x);
        SetD2Ry2du22(d2u2[0].y);
        SetD3Rx2du23(d3u2[0].x);
        SetD3Ry2du23(d3u2[0].y);
        SetD4Rx2du24(d4u2[0].x);
        SetD4Ry2du24(d4u2[0].y);

        d1u1.Dispose();
        d2u1.Dispose();
        d3u1.Dispose();
        d4u1.Dispose();
        d1u2.Dispose();
        d2u2.Dispose();
        d3u2.Dispose();
        d4u2.Dispose();
    }


    public void Dispose()
    {
        if (ctrl.IsCreated)  ctrl.Dispose();

        if (beta1.IsCreated) beta1.Dispose();
        if (beta2.IsCreated) beta2.Dispose();
        if (beta3.IsCreated) beta3.Dispose();
        if (beta4.IsCreated) beta4.Dispose();

        if (knots.IsCreated) knots.Dispose();
        if (knots1.IsCreated) knots1.Dispose();
        if (knots2.IsCreated) knots2.Dispose();
        if (knots3.IsCreated) knots3.Dispose();
        if (knots4.IsCreated) knots4.Dispose();

        if (points.IsCreated) points.Dispose();
        if (derivative1.IsCreated) derivative1.Dispose();
    }

    //     public void Dispose()
//     {
//         if (ctrl.IsCreated) ctrl.Dispose();
//         if (knots.IsCreated) knots.Dispose();
//         if (points.IsCreated) points.Dispose();
//         if (derivative1.IsCreated) derivative1.Dispose();

//     }


    // Setter
    public void SetRx1(float v) { rx1 = v; }
    public void SetRy1(float v) { ry1 = v; }
    public void SetD1Rx1du11(float v) { d1Rx1du11 = v; }
    public void SetD1Ry1du11(float v) { d1Ry1du11 = v; }
    public void SetD2Rx1du12(float v) { d2Rx1du12 = v; }
    public void SetD2Ry1du12(float v) { d2Ry1du12 = v; }
    public void SetD3Rx1du13(float v) { d3Rx1du13 = v; }
    public void SetD3Ry1du13(float v) { d3Ry1du13 = v; }
    public void SetD4Rx1du14(float v) { d4Rx1du14 = v; }
    public void SetD4Ry1du14(float v) { d4Ry1du14 = v; }

    public void SetRx2(float v) { rx2 = v; }
    public void SetRy2(float v) { ry2 = v; }
    public void SetD1Rx2du21(float v) { d1Rx2du21 = v; }
    public void SetD1Ry2du21(float v) { d1Ry2du21 = v; }
    public void SetD2Rx2du22(float v) { d2Rx2du22 = v; }
    public void SetD2Ry2du22(float v) { d2Ry2du22 = v; }
    public void SetD3Rx2du23(float v) { d3Rx2du23 = v; }
    public void SetD3Ry2du23(float v) { d3Ry2du23 = v; }
    public void SetD4Rx2du24(float v) { d4Rx2du24 = v; }
    public void SetD4Ry2du24(float v) { d4Ry2du24 = v; }

    // Getter
    public float GetRx1() => rx1;
    public float GetRy1() => ry1;
    public float GetD1Rx1du11() => d1Rx1du11;
    public float GetD1Ry1du11() => d1Ry1du11;
    public float GetD2Rx1du12() => d2Rx1du12;
    public float GetD2Ry1du12() => d2Ry1du12;
    public float GetD3Rx1du13() => d3Rx1du13;
    public float GetD3Ry1du13() => d3Ry1du13;
    public float GetD4Rx1du14() => d4Rx1du14;
    public float GetD4Ry1du14() => d4Ry1du14;

    public float GetRx2() => rx2;
    public float GetRy2() => ry2;
    public float GetD1Rx2du21() => d1Rx2du21;
    public float GetD1Ry2du21() => d1Ry2du21;
    public float GetD2Rx2du22() => d2Rx2du22;
    public float GetD2Ry2du22() => d2Ry2du22;
    public float GetD3Rx2du23() => d3Rx2du23;
    public float GetD3Ry2du23() => d3Ry2du23;
    public float GetD4Rx2du24() => d4Rx2du24;
    public float GetD4Ry2du24() => d4Ry2du24;

    public NativeArray<float> GetKnots() => knots;
    // public NativeArray<float> GetKnots1() => knots1;
    public NativeArray<float2> GetBeta1() => beta1;

    public float2[] GetPointsManaged()
    {
        float2[] arr = new float2[N];
        for (int i = 0; i < N; i++)
            arr[i] = points[i];
        return arr;
    }

}


    // private int FindSpan(float u, int degree, int n, NativeArray<float> knots)
    // {
    //     if (u >= knots[n + 1])
    //         return n;

    //     int low = degree;
    //     int high = n + 1;
    //     int mid = (low + high) / 2;

    //     while (u < knots[mid] || u >= knots[mid + 1])
    //     {
    //         if (u < knots[mid]) high = mid;
    //         else low = mid;
    //         mid = (low + high) / 2;
    //     }

    //     return mid;
    // }

    // points[i] = CalculateBsplinePoint(u, ctrl, k, knots);
    // public float2 CalculateBsplinePoint(float u, NativeArray<float2> ctrl, int degree, NativeArray<float> knots)
    // {
    //     return DeBoorEvaluate(ctrl, u, degree, knots);
    // }

    // private float2 DeBoorEvaluate(NativeArray<float2> ctrl, float u, int degree, NativeArray<float> knots)
    // {
    //     int n = ctrl.Length - 1;

    //     int span = FindSpan(u, degree, ctrl.Length - 1, knots);

    //     float2[] d = new float2[degree + 1];

    //     for (int j = 0; j <= degree; j++)
    //     {
    //         d[j] = ctrl[span - degree + j];
    //     }

    //     for (int r = 1; r <= degree; r++)
    //     {
    //         for (int j = degree; j >= r; j--)
    //         {
    //             int i = span - degree + j;
    //             float denom = knots[i + degree - r + 1] - knots[i];
    //             float alpha = (denom == 0) ? 0 : (u - knots[i]) / denom;

    //             d[j] = math.lerp(d[j - 1], d[j], alpha);
    //         }
    //     }

    //     return d[degree];
    // }