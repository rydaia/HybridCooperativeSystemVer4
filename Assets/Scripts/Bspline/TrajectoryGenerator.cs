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

    // 目標点の過去データ 毎フレーム更新
    public NativeArray<float2> pastNative;   // 
    // Bスプライン近似させる時の制御点データ
    // 判定条件を満たしたら更新
    public NativeArray<float2> controlPointsNative;   // 
    public NativeArray<float2> resampledNative;   // 
    public NativeArray<float2> smoothedNative; // \

    public float smax_past;
    public float smax_future; 

    public int Np;
    public int Nf;
    public int Nre;
    public int Nsm;
    public int N;
    public int k;

    public float ds, dt;

    private bool isUpdateCPdataFlag;


    // コントラスタ
    public TrajectoryGenerator(CalculationManager calc)
    {
        this.calc = calc;
    }

    public void Initialize()
    {
        this.ds = calc.ds;
        this.dt = calc.dt;

        // 初回は必ず更新
        isUpdateCPdataFlag = false;

        k = 5;

        smax_past = 20.0f; // [m]
        smax_future = 4.0f; // [m]

        // 刻み幅 0.01[m] = 1[cm]
        Np = Mathf.FloorToInt(smax_past / 0.01f) + 1;
        Nf = Mathf.FloorToInt(smax_future / 0.1f);

        calc.cpQueue.Initialize(dt, calc);

        calc.cpSmooth.Initialize(Np, calc);

        calc.cpResample.Initialize(smax_past);

        // 再サンプリングした際のサイズ
        Nre = calc.cpResample.GetN();


        // 近似後のサイズ
        N = Mathf.FloorToInt(smax_past / ds) + 1;
        // スムージング済みデータのサイズ
        Nsm = calc.cpSmooth.GetNsm();

        // mergedNative = new NativeArray<float2>(Np + Nf, Allocator.Persistent);
        pastNative = new NativeArray<float2>(Np, Allocator.Persistent);
        controlPointsNative = new NativeArray<float2>(Np, Allocator.Persistent);
        smoothedNative = new NativeArray<float2>(Nsm, Allocator.Persistent);
        resampledNative = new NativeArray<float2>(Nre, Allocator.Persistent);
        // bsplineNative = new NativeArray<float2>(N, Allocator.Persistent);

        Debug.Log($"過去データの総数:{Np}");
        Debug.Log($"smax_past:{smax_past}");
        Debug.Log($"距離等間隔に再サンプリングする際のデータ総数:{Nre}");
        Debug.Log($"スムージングデータの総数:{Nsm}");
        Debug.Log($"bスプライン曲線の総数:{N}");

    }

    public void Reset()
    {
        // 初回は必ず更新
        isUpdateCPdataFlag = false;

        calc.cpQueue.Initialize(dt, calc);
        calc.cpSmooth.Initialize(Np, calc);

        calc.cpResample.Initialize(smax_past);
        
        InitializeCurve();

        Debug.Log($"Reset TrajectoryGenerator");
    }

    public void InitializeCurve()
    {
        pastNative = calc.cpQueue.GetPastNative();
        controlPointsNative = pastNative;
        smoothedNative = calc.cpSmooth.ApplyMovingAverage(controlPointsNative);
        resampledNative = calc.cpResample.ResampleByDistance(smoothedNative);
        calc.bsplineGeometry.Approximate(GetResampledManaged());

        calc.bsplineGeometry.Derivative1AllPoints();

        calc.psFinder.RebuildCache();
    }

    public void GenerateBSplineCurve()
    {

        using (generateBSplineMarker.Auto())
        {
            using (updateControlPointsMarker.Auto())
            {
                // 過去データは更新し続ける
                UpdatePastData();
            }

            // 制御点として用いるデータは距離判定で決める
            // 現在の第一連結点の座標(x1, y1)が曲線の端の点と近くなった時、
            // 判定条件を満たした時、その時の過去データを制御点データとして更新する
            DetermineUpdateControlePoint();

            SetIsUpdateCPdataFlag(true);

            if(GetIsUpdateCPFlag())
            {
                // Debug.Log($"Bスプライン曲線を再生成します.");
                UpdateControlPoints();  

                using (smoothControlPointsMarker.Auto())
                {
                    smoothedNative = calc.cpSmooth.ApplyMovingAverage(controlPointsNative);
                }

                using (resampleControlPointsMarker.Auto())
                {
                    resampledNative = calc.cpResample.ResampleByDistance(smoothedNative);
                }

                // Bスプライン曲線 近似
                using (BSplineInterpolatorMarker.Auto())
                {
                    calc.bsplineGeometry.Approximate(GetResampledManaged());
                }

                using(BSplineDerivativeMarker.Auto())
                {
                    calc.bsplineGeometry.Derivative1AllPoints();
                }
            }
        }
    }

    public void UpdatePastData()
    {
        Vector2 Tp = calc.targetPointState.GetPosition();
        float theta = calc.targetPointState.GetTheta();
        float v1 = calc.targetPointState.GetV1();
        float kappa = calc.targetPointState.GetKappa(); // dθ/ds

        calc.cpQueue.Update(Tp, theta, v1, kappa);
    }

    // 現在のpastデータをcontrolePointsデータにコピー
    private void UpdateControlPoints()
    {

        pastNative = calc.cpQueue.GetPastNative();

        for (int i = 0; i < Np; i++)
        {
            controlPointsNative[i] = pastNative[i];
        }
    }

    // 制御点を更新するかしない判定
    private void DetermineUpdateControlePoint()
    {

        //現在の第一連結点の座標(x1,y1)を取得
        Vector2 pos = calc.vehicleRobotState.GetMidpointBetweenRearWheelsOfFV();

        // 現在の制御点データの端点を取得
        Vector2 pos1 = controlPointsNative[GetNp() - 1];

        float diff = math.distance(pos, pos1);

        float eps = 0.5f; // 0.1[m]

        if(diff < eps)
        {
            SetIsUpdateCPdataFlag(true);
        }
        else
        {
            SetIsUpdateCPdataFlag(false);
        }
    }

    public float2[] GetResampledManaged()
    {
        float2[] arr = new float2[Nre];
        for (int i = 0; i < Nre; i++)
            arr[i] = resampledNative[i];
        return arr;
    }


    public bool GetIsUpdateCPFlag()
    {
        return isUpdateCPdataFlag;
    }

    public void SetIsUpdateCPdataFlag(bool v)
    {
        isUpdateCPdataFlag = v;
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
        return Nre;
    }

    public float GetDs()
    {
        return ds;
    }
}
