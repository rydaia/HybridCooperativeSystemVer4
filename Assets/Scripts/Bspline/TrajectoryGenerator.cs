using UnityEngine;
using Unity.Profiling;
using Unity.Mathematics;
using Unity.Collections;   
using Unity.Jobs;        
using Unity.Burst;    
using System.Collections.Generic;

// 目標点の現在座標を取得

// 制御点の生成
// 過去の軌跡の点集合と現時点のu1(前身速度), u2(ステアリング速度)による未来の予測軌道の点集合からなる制御点
// それらの制御点の集合をスムージングにより滑らかにする
// 最終的な制御点集合からB-Spline補完、曲線の生成

//　制御点の集合はキュー(FIFD)による実装 1からNまで
// 未来の予測軌道の点集合はどう決まる？
// pure class
public class TrajectoryGenerator
{

    static ProfilerMarker generateBSplineMarker =
        new ProfilerMarker("TrajectoryGenerator.GenerateBSpline");

    static ProfilerMarker updateControlPointsMarker = 
        new ProfilerMarker("TrajectoryGenerator.UpdateControlPoints");

    static ProfilerMarker resampleControlPointsMarker = 
        new ProfilerMarker("TrajectoryGenerator.ResampleControlPoints");

    static ProfilerMarker smoothControlPointsMarker = 
        new ProfilerMarker("TrajectoryGenerator.SmoothControlPoints");

    static ProfilerMarker BSplineInterpolatorMarker = 
        new ProfilerMarker("TrajectoryGenerator.BSplineInterpolator");

    static ProfilerMarker BSplineDerivativeMarker = 
        new ProfilerMarker("TrajectoryGenerator.BSplineDerivative");

    public CalculationManager calc;

    public Vector2[] points; // Bスプライン結果
    public NativeArray<float2> mergedNative;   // 
    public NativeArray<float2> resampledNative;   // 
    public NativeArray<float2> smoothedNative; // 
    public NativeArray<float2> resampled2Native;   // 
    public NativeArray<float2> bsplineNative; // 
    public NativeArray<float2> bsplineDerivativeNative; // 


    public float smax_past; 
    public float smax_future; 

    public int Np;
    public int Nf;
    public int Nctrl;
    public int Nsm;
    public int N;
    public int k;


    public float ds, dt;


    // コントラスタ
    public TrajectoryGenerator(CalculationManager calc)
    {
        this.calc = calc;
    }

    public void Initialize()
    {
        this.ds = calc.ds;
        this.dt = calc.dt;

        k = 5;

        smax_past = 10.0f; // [m]
        smax_future = 2.0f; // [m]

        Np = Mathf.FloorToInt(smax_past / 0.01f) + 1;
        Nf = Mathf.FloorToInt(smax_future / 0.01f);

        calc.cpQueue.Initialize(Np, Nf, dt, calc);
        calc.cpResample.Initialize(smax_past, smax_future);

        // 再サンプリングした際のサイズ
        Nctrl = calc.cpResample.GetN();

        calc.cpSmooth.Initialize(Nctrl, calc);

        calc.cpQueue.Merge();

        // 近似後のサイズ
        N = Mathf.FloorToInt(smax_past / ds + smax_future / ds) + 1;

        // スムージング済みデータのサイズ
        Nsm = calc.cpSmooth.GetNsm();

        mergedNative = new NativeArray<float2>(Np + Nf, Allocator.Persistent);
        smoothedNative = new NativeArray<float2>(Nsm, Allocator.Persistent);
        resampled2Native = new NativeArray<float2>(Nctrl, Allocator.Persistent);
        bsplineNative = new NativeArray<float2>(N, Allocator.Persistent);

        Debug.Log($"過去データの総数:{Np}");
        Debug.Log($"未来データの総数:{Nf}");
        Debug.Log($"マージデータの総数:{Np+Nf}");
        Debug.Log($"smax_past:{smax_past}, smax_furure:{smax_future}");
        Debug.Log($"距離等間隔に再サンプリングする際のデータ総数:{Nctrl}");
        Debug.Log($"スムージングデータの総数:{Nsm}");
        Debug.Log($"bスプライン曲線の総数:{N}");

    }

    public void InitializeCurve()
    {
        mergedNative = calc.cpQueue.GetMergedNative();
        smoothedNative = calc.cpSmooth.ApplyMovingAverage(mergedNative);
        resampled2Native = calc.cpResample.ResampleByDistance(smoothedNative);
        bsplineNative = calc.bsplineGeometry.Approximate(resampled2Native);


        bsplineDerivativeNative = calc.bsplineGeometry.DerivativeAllPoints();
    }

    public void GenerateBSplineCurve()
    {

        using (generateBSplineMarker.Auto())
        {
            using (updateControlPointsMarker.Auto())
            {
                UpdateControlPoints();
                // ここで merged を取得
                mergedNative = calc.cpQueue.GetMergedNative();
            }

            using (smoothControlPointsMarker.Auto())
            {
                // スムージング Burst化+Job
                // smoothedNative = calc.cpSmooth.ApplyMovingAverage(resampledNative);
                smoothedNative = calc.cpSmooth.ApplyMovingAverage(mergedNative);
            }

            using (resampleControlPointsMarker.Auto())
            {
                resampled2Native = calc.cpResample.ResampleByDistance(smoothedNative);
            }

            // Bスプライン曲線 近似
            using (BSplineInterpolatorMarker.Auto())
            {
                // Bスプライン補完
                bsplineNative = calc.bsplineGeometry.Approximate(resampled2Native);

            }

            using(BSplineDerivativeMarker.Auto())
            {
                bsplineDerivativeNative = calc.bsplineGeometry.DerivativeAllPoints();
                // Debug.Log($"bsplineNative[0]:{bsplineNative[0]}");
                // Debug.Log($"bsplineDerivativeNative[0]:{bsplineDerivativeNative[0]}");
                // Debug.Log($"bsplineNative[1]:{bsplineNative[1]}");
                // Debug.Log($"bsplineDerivativeNative[1]:{bsplineDerivativeNative[1]}");
                // Debug.Log($"bsplineNative[100000]:{bsplineNative[100000]}");
                // Debug.Log($"bsplineDerivativeNative[100000]:{bsplineDerivativeNative[100000]}");
            }
        }
    }

    public void UpdateControlPoints()
    {


        Vector2 Tp = calc.targetPointState.GetPosition();
        float theta = calc.targetPointState.getTheta();
        float v1 = calc.targetPointState.getV1();
        float kappa = calc.targetPointState.getKappa(); // dθ/ds

        calc.cpQueue.Update(Tp, theta, v1, kappa);
    }

    public NativeArray<float2> GetBsplineNative()
    {
        return bsplineNative;
    }

    public NativeArray<float2> GetBsplineDerivativeNative()
    {
        return bsplineDerivativeNative;
    }


    public int GetN()
    {
        return N;
    }

    public int GetK()
    {
        return k;
    }

    public int GetNctrl()
    {
        return Nctrl;
    }

    public float GetDs()
    {
        return ds;
    }
}
