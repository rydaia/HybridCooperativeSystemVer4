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
    // public NativeArray<float2> mergedNative;   // 
    public NativeArray<float2> pastNative;   // 
    public NativeArray<float2> resampledNative;   // 
    public NativeArray<float2> smoothedNative; // 
    // public NativeArray<float2> resampled2Native;   // 
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

        smax_past = 20.0f; // [m]
        smax_future = 4.0f; // [m]

        Np = Mathf.FloorToInt(smax_past / 0.01f) + 1;
        Nf = Mathf.FloorToInt(smax_future / 0.01f);

        calc.cpQueue.Initialize(dt, calc);
        calc.cpResample.Initialize(smax_past);

        // 再サンプリングした際のサイズ
        Nctrl = calc.cpResample.GetN();

        calc.cpSmooth.Initialize(Nctrl, calc);

        // calc.cpQueue.Merge();

        // 近似後のサイズ
        N = Mathf.FloorToInt(smax_past / ds) + 1;
        // N = Mathf.FloorToInt(smax_past / ds + smax_future / ds) + 1;

        // スムージング済みデータのサイズ
        Nsm = calc.cpSmooth.GetNsm();

        // mergedNative = new NativeArray<float2>(Np + Nf, Allocator.Persistent);
        pastNative = new NativeArray<float2>(Np, Allocator.Persistent);
        smoothedNative = new NativeArray<float2>(Nsm, Allocator.Persistent);
        resampledNative = new NativeArray<float2>(Nctrl, Allocator.Persistent);
        bsplineNative = new NativeArray<float2>(N, Allocator.Persistent);

        Debug.Log($"過去データの総数:{Np}");
        // Debug.Log($"未来データの総数:{Nf}");
        // Debug.Log($"マージデータの総数:{Np+Nf}");
        Debug.Log($"smax_past:{smax_past}");
        // Debug.Log($"smax_past:{smax_past}, smax_furure:{smax_future}");
        Debug.Log($"距離等間隔に再サンプリングする際のデータ総数:{Nctrl}");
        Debug.Log($"スムージングデータの総数:{Nsm}");
        Debug.Log($"bスプライン曲線の総数:{N}");

    }

    public void InitializeCurve()
    {
        // mergedNative = calc.cpQueue.GetMergedNative();
        pastNative = calc.cpQueue.GetPastNative();
        smoothedNative = calc.cpSmooth.ApplyMovingAverage(pastNative);
        // Debug.Log($"a");
        resampledNative = calc.cpResample.ResampleByDistance(smoothedNative);
        // Debug.Log($"b");
        // bsplineNative = calc.bsplineGeometry.Approximate(resampledNative);
        // Debug.Log($"c");
        bsplineDerivativeNative = calc.bsplineGeometry.DerivativeAllPoints();
        // Debug.Log($"d");
    }

    public void GenerateBSplineCurve()
    {

        using (generateBSplineMarker.Auto())
        {
            using (updateControlPointsMarker.Auto())
            {
                UpdateControlPoints();
                // ここで merged を取得
                // mergedNative = calc.cpQueue.GetMergedNative();
                pastNative = calc.cpQueue.GetPastNative();
                // pastNative = calc.cpQueue.GetPastPointsManaged();

                // Debug.Log($"pastNative[0]={pastNative[0]}");
                // Debug.Log($"pastNative[1]={pastNative[1]}");
                // Debug.Log($"pastNative[2]={pastNative[2]}");
                // Debug.Log($"pastNative[3]={pastNative[3]}");
                // Debug.Log($"pastNative[4]={pastNative[4]}");
                // Debug.Log($"-----------------------------");
                // Debug.Log($"pastNative[996]={pastNative[996]}");
                // Debug.Log($"pastNative[997]={pastNative[997]}");
                // Debug.Log($"pastNative[998]={pastNative[998]}");
                // Debug.Log($"pastNative[999]={pastNative[999]}");
                // Debug.Log($"pastNative[1000]={pastNative[1000]}");

            }

            using (smoothControlPointsMarker.Auto())
            {
                // スムージング Burst化+Job
                // smoothedNative = calc.cpSmooth.ApplyMovingAverage(resampledNative);
                smoothedNative = calc.cpSmooth.ApplyMovingAverage(pastNative);

                // Debug.Log($"smoothedNative[0]={smoothedNative[0]}");
                // Debug.Log($"smoothedNative[1]={smoothedNative[1]}");
                // Debug.Log($"smoothedNative[2]={smoothedNative[2]}");
                // Debug.Log($"smoothedNative[3]={smoothedNative[3]}");
                // Debug.Log($"smoothedNative[4]={smoothedNative[4]}");
            }

            using (resampleControlPointsMarker.Auto())
            {
                resampledNative = calc.cpResample.ResampleByDistance(smoothedNative);

                // Debug.Log($"resampledNative[0]={resampledNative[0]}");
                // Debug.Log($"resampledNative[1]={resampledNative[1]}");
                // Debug.Log($"resampledNative[2]={resampledNative[2]}");
                // Debug.Log($"resampledNative[3]={resampledNative[3]}");
                // Debug.Log($"resampledNative[4]={resampledNative[4]}");
            }

            // Bスプライン曲線 近似
            using (BSplineInterpolatorMarker.Auto())
            {
                // Bスプライン補完
                bsplineNative = calc.bsplineGeometry.Approximate(resampledNative);

                // Debug.Log($"bsplineNative[0]={bsplineNative[0]}");
                // Debug.Log($"bsplineNative[1]={bsplineNative[1]}");
                // Debug.Log($"bsplineNative[2]={bsplineNative[2]}");
                // Debug.Log($"bsplineNative[3]={bsplineNative[3]}");
                // Debug.Log($"bsplineNative[4]={bsplineNative[4]}");

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

    public int GetNp()
    {
        return Np;
    }

    public int GetNf()
    {
        return Nf;
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
