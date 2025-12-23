using System;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Globalization;
using System.Text;  
using Unity.Mathematics;
using Unity.Profiling;



public class CalculationManager : MonoBehaviour
{

    // MonoBehaviour
    [Header("計算マネージャー")]
    public SimulationManager sim;

    // pure class
    [Header("B-Spline幾何学インスタンス")]
    public TrajectoryGenerator trajectoryGenerator;

    public TargetPointCtrl targetPointCtrl;

    public ControlPointQueue cpQueue;
    public ControlPointSmoother cpSmooth;
    public ControlPointResampler cpResample;

    public BsplineGeometry bsplineGeometry;    
    public PathKinematics pathKinematics;    
    public VehicleKinematics vehicleKinematics;   

    public TargetPointState targetPointState;
    public TargetPointDynamics targetPointDynamics;

    public VehicleRobotState vehicleRobotState;
    public VehicleRobotDynamics vehicleRobotDynamics;
    public VehicleParameters vehicleRobotParms;

    public PsFinder psFinder;


    [Header("debug用")]
    // private float2[] debugMergedPoints;
    private float2[] debugResampledPoints;
    // private float2[] debugResampledPoints2;
    private float2[] debugSmoothedPoints;
    private float2[] debugPastPoints;
    // private float2[] debugFuturePoints;
    private float2[] debugPoints;
    public int debugNp;
    // public int debugNf;
    public int debugNs;
    // public int debugNd;
    public int debugN;

    public event Action OnInitialized;

    // クラスフィールドの冒頭に追加
    private StringBuilder csvBuffer = new StringBuilder(1024 * 128); // 128KBバッファ
    private const int FLUSH_INTERVAL = 1000; // 1000ステップごとにファイル出力 何ステップごとに書き込むか
        
    public float dt, ds;

    [System.Serializable]
    public struct SimulationData
    {
        public float time;
        public float s;
        public float x, y; // 目標点の座標
        public float theta; // 目標点の姿勢角

        public float v1; // 第一車両 前進速度
        public float v2; // 第一車両 ステアリング角速度

        public float x1, y1; // 目標点の座標
        public float x2, y2; // 目標点の座標
        public float phi1, phi2; // 目標点の姿勢角
        public float theta1, theta2, theta3; // 目標点の姿勢角
        public float thetaT1, thetaT2, thetaP2d; // 目標点の姿勢角

        public float u1, u2, u3, u4; // 第一車両 前進速度

        public float w1, w2, w3; // 第一車両 前進速度

        public float d1; 

        public int u1Index, u2Index;
        public float u1Float, u2Float;

        public float rx1, ry1;
        public float rx2, ry2;
        public float cs1, cs2;
        public float d1rx1du11, d1ry1du11;
        public float d2rx1du12, d2ry1du12;
        public float d3rx1du13, d3ry1du13;
        public float d4rx1du14, d4ry1du14;
        // public float d1rx2du21, d1ry2du21;
        // public float d2rx2du22, d2ry2du22;
        // public float d3rx2du23, d3ry2du23;
        // public float d4rx2du24, d4ry2du24;
    }

    void Awake()
    {

        vehicleRobotParms = new VehicleParameters();

        // TargetPointState のインスタンス生成
        targetPointDynamics = new TargetPointDynamics();
        targetPointState = new TargetPointState(targetPointDynamics);

        vehicleRobotDynamics = new VehicleRobotDynamics(vehicleRobotParms);
        vehicleRobotState = new VehicleRobotState(vehicleRobotDynamics, vehicleRobotParms);

        // ControlPointQueue のインスタンス生成
        cpQueue = new ControlPointQueue();
        // ControlPointResampler のインスタンス生成
        cpResample = new ControlPointResampler();
        // ControlPointSmoother のインスタンス生成c
        cpSmooth = new ControlPointSmoother();
        // BSplineCurve のインスタンス生成
        // TrajectoryGenerator のインスタンス生成
        trajectoryGenerator = new TrajectoryGenerator(this); //　ここのthisはCalculationManagerクラスの自己インスタンス

        psFinder = new PsFinder(vehicleRobotState, trajectoryGenerator, vehicleRobotParms, sim);

        bsplineGeometry = new BsplineGeometry();

        // psFinder = new PsFinder(vehicleRobotState, trajectoryGenerator, bsplineGeometry, vehicleRobotParms);

        // pathKinematics = new PathKinematics();

        // vehicleKinematics  = new VehicleKinematics(

        // );
    }
    
    
    void Start()
    {
        InitializeCalculateSimulation();
    }
    
    public void InitializeCalculateSimulation()
    {

        dt = 0.01f;
        ds = 0.001f; // 0.001[m], 1.0[mm]

        targetPointCtrl.Initialize();

        targetPointState.Initialize(targetPointCtrl, vehicleRobotState);
        vehicleRobotState.Initialize(vehicleKinematics);

        trajectoryGenerator.Initialize();

        bsplineGeometry.Initialize(trajectoryGenerator, psFinder);

        psFinder.Initialize(bsplineGeometry);

        pathKinematics.Initialize(trajectoryGenerator, bsplineGeometry);
        vehicleKinematics.Initialize(            
            targetPointState, 
            vehicleRobotState, 
            vehicleRobotParms, 
            bsplineGeometry, 
            pathKinematics);

        trajectoryGenerator.InitializeCurve();
        

        // ここでイベント発火
        OnInitialized?.Invoke();
    }

    void Update()
    {
        if (sim.isSimulationRunning)
        {
            // dt = Time.deltaTime;
            dt = 0.01f;

            PerformCalculationStep(dt);
        }

        // 監視用に cpQueue の値をコピー
        if(sim.enableDebugOutput)
        {
            debugNp = cpQueue.GetNp();
            // debugNf = cpQueue.GetNf();
            debugNs = cpSmooth.GetNsm();
            debugN = trajectoryGenerator.GetN();

            // debugMergedPoints = cpQueue.GetMergedPointsManaged();
            debugResampledPoints = cpResample.GetResampledPointsManaged(trajectoryGenerator.resampledNative);
            // debugResampledPoints2 = cpResample.GetResampledPointsManaged(trajectoryGenerator.resampled2Native);
            debugSmoothedPoints = cpSmooth.GetSmoothedPointsManaged();
            debugPastPoints   = cpQueue.GetPastPointsManaged();
            // debugFuturePoints = cpQueue.GetFuturePointsManaged();
            // debugPoints = bsplineGeometry.GetPointsManaged();
        }
    }
    
    
    void PerformCalculationStep(float _dt)
    {
        float time = targetPointState.getTime();

        // 進捗表示
        if (sim.currentStep % 1000 == 0)
        {
            Debug.Log($"time:{time}");
        }

        if (sim.currentStep >= sim.totalSteps)
        {
            // Debug.Log("シミュレーション計算終了");
            sim.EndSimulation();
        }

        if (time >= sim.tMax)
        {
            // Debug.Log($"timeがtMaxに到達しました。time:{time} q1:{q1}");
            sim.EndSimulation();
            return;
        }

        // if(targetPointState.getTime() > 0.11f)
        // {
        //     Debug.Log($"10フレーム");
        //     sim.EndSimulation();
        //     return;
        // };

        // Debug.Log($"current.t:{targetPointState.getTime()}, current.s:{targetPointState.getS()}, current.v1:{targetPointState.getV1()}, current.x:{targetPointState.getX()}, current.s:{targetPointState.getY()}");

        // ここで入力の読み込みたい
        targetPointCtrl.ReadInput(Mathf.FloorToInt(targetPointState.getTime() / _dt)); 
        
        SimulationData data = new SimulationData();
        
        //ここで曲線の生成
        // TrajectoryGeneratorクラス呼び出し
        trajectoryGenerator.GenerateBSplineCurve();

        // if(targetPointState.getTime() > 30.01f)
        // {
        //     Debug.Log($"time:{time}");
        //     sim.StopSimulation();
        // };

        // ここにPs探索
        psFinder.StepPsFinder();

        bsplineGeometry.CalculationBsplineGeometry();

        pathKinematics.CalculationPathKinematics();

        vehicleKinematics.CalculationVehicleKinematics();


        // 目標点の状態更新
        // Stateクラスから関数呼び出し　updateTargetPointState()
        targetPointState.updateTargetPointState(_dt);

        vehicleRobotState.updateVehicleRobotState(_dt);



        if (sim.enableOutputSimulationData)
        {

            // データ保存
            data.time = targetPointState.getTime();
            data.s = targetPointState.getS();
            data.x = targetPointState.getX();
            data.y = targetPointState.getY();
            data.theta = targetPointState.getTheta();
            data.v1 = targetPointState.getV1();
            data.v2 = targetPointState.getV2();

            data.x1 = vehicleRobotState.GetX1();
            data.y1 = vehicleRobotState.GetY1();
            data.phi1 = vehicleRobotState.GetPhi1();
            data.phi2 = vehicleRobotState.GetPhi2();
            data.theta1 = vehicleRobotState.GetTheta1();
            data.theta2 = vehicleRobotState.GetTheta2();
            data.theta3 = vehicleRobotState.GetTheta3();

            data.thetaT1 = pathKinematics.GetThetaT1();
            data.thetaT2 = pathKinematics.GetThetaT2();
            data.thetaP2d = pathKinematics.GetThetaP2d();

            data.x2 = vehicleRobotState.GetX2();
            data.y2 = vehicleRobotState.GetY2();

            data.u1 = vehicleRobotState.GetU1();
            data.u2 = vehicleRobotState.GetU2();
            data.u3 = vehicleRobotState.GetU3();
            data.u4 = vehicleRobotState.GetU4();


            data.w1 = vehicleKinematics.GetW1();
            data.w2 = vehicleKinematics.GetW2();
            data.w3 = vehicleKinematics.GetW3();

            data.u1Index = psFinder.GetU1Index();
            data.u2Index = psFinder.GetU2Index();

            data.u1Float = psFinder.GetU1();
            data.u2Float = psFinder.GetU2();

            data.rx1 = bsplineGeometry.GetRx1();
            data.ry1 = bsplineGeometry.GetRy1();
            data.rx2 = bsplineGeometry.GetRx2();
            data.ry2 = bsplineGeometry.GetRy2();
            data.cs1 = pathKinematics.GetCs1();
            data.cs2 = pathKinematics.GetCs2();

            data.d1rx1du11 = bsplineGeometry.GetD1Rx1du11();
            data.d1ry1du11 = bsplineGeometry.GetD1Ry1du11();
            data.d2rx1du12 = bsplineGeometry.GetD2Rx1du12();
            data.d2ry1du12 = bsplineGeometry.GetD2Ry1du12();
            data.d3rx1du13 = bsplineGeometry.GetD3Rx1du13();
            data.d3ry1du13 = bsplineGeometry.GetD3Ry1du13();
            data.d4rx1du14 = bsplineGeometry.GetD4Rx1du14();
            data.d4ry1du14 = bsplineGeometry.GetD4Ry1du14();

            data.d1 = vehicleKinematics.GetD1();

            // data.d1rx2du21 = bsplineGeometry.GetD1Rx2du21();
            // data.d1ry2du21 = bsplineGeometry.GetD1Ry2du21();
            // data.d2rx2du22 = bsplineGeometry.GetD2Rx2du22();
            // data.d2ry2du22 = bsplineGeometry.GetD2Ry2du22();
            // data.d3rx2du23 = bsplineGeometry.GetD3Rx2du23();
            // data.d3ry2du23 = bsplineGeometry.GetD3Ry2du23();
            // data.d4rx2du24 = bsplineGeometry.GetD4Rx2du24();
            // data.d4ry2du24 = bsplineGeometry.GetD4Ry2du24();

            // データをバッファに追加
            AppendCsvLine(data);

            // 一定間隔でファイルに出力
            if (sim.currentStep % FLUSH_INTERVAL == 0)
            {
                sim.output.Write(csvBuffer.ToString());
                csvBuffer.Clear();
                sim.output.Flush(); // バッファをディスクへ反映
            }
        }

        // 状態更新
        sim.currentStep++;
        
        // 進捗表示
        if (sim.currentStep % 1000 == 0)
        {
            Debug.Log($"計算進捗: {sim.currentStep}/{sim.totalSteps} time:{time}");
        }
    }

    // StringBuilderを使って1行分を構築
    private void AppendCsvLine(SimulationData d)
    {
        csvBuffer.AppendFormat(CultureInfo.InvariantCulture,
            "{0:F6}," +
            "{1:F6},{2:F6}," +
            "{3:F6}," +
            "{4:F6},{5:F6}," +
            "{6:F6},{7:F6}," +
            "{8:F6},{9:F6}," +
            "{10:F6},{11:F6},{12:F6}," +
            "{13:F6},{14:F6},{15:F6}," +
            "{16:F6},{17:F6}," +
            "{18:F6},{19:F6}," +
            "{20:F6},{21:F6}," +
            "{22:F6},{23:F6}," + 
            "{24:F6},{25:F6}," +
            "{26:F6},{27:F6}," +
            "{28:F6},{29:F6}," + 
            "{30:F6},{31:F6},{32:F6},{33:F6},{34:F6},{35:F6},{36:F6},{37:F6},{38:F6},{39:F6},{40:F6},{41:F6},{42:F6},{43:F6},{44:F6}\n",
            d.time,
            d.s,
            d.x, d.y,
            d.theta,
            d.v1, d.v2,
            d.x1, d.y1,
            d.phi1, d.theta1,
            d.phi2, d.theta2, d.theta3,
            d.x2, d.y2,
            d.u1, d.u2, d.u3, d.u4,
            d.u1Index, d.u2Index,
            d.u1Float, d.u2Float,
            d.rx1, d.ry1,
            d.d1rx1du11, d.d1ry1du11,
            d.d2rx1du12, d.d2ry1du12,
            d.d3rx1du13, d.d3ry1du13,
            d.d4rx1du14, d.d4ry1du14,
            d.rx2, d.ry2,
            d.cs1, d.cs2,
            d.thetaT1, d.thetaT2, d.thetaP2d,
            d.d1, 
            d.w1, d.w2, d.w3
        );
    }


    public void FlushRemainingBuffer()
    {
        if (csvBuffer.Length > 0 && sim.output != null)
        {
            sim.output.Write(csvBuffer.ToString());
            csvBuffer.Clear();
            sim.output.Flush();
        }
    }

    void OnDestroy()
    {
        cpQueue.Dispose();
        cpResample.Dispose();
        cpSmooth.Dispose();
        // bsplineGeometry.Dispose();
        psFinder.Dispose();
    }

    public void OutputDebug()
    {
        if(sim.enableDebugOutput)
        {
            // OutputCSV.WriteBSplineCurve(debugPastPoints, debugFuturePoints, debugMergedPoints, debugResampledPoints, debugSmoothedPoints, debugResampledPoints2, debugPoints, ds, targetPointState.getTime());
            OutputCSV.WriteBSplineCurve(
                debugPastPoints, 
                debugSmoothedPoints, 
                debugResampledPoints, 
                bsplineGeometry.GetArrayU(),
                bsplineGeometry.GetFrontPoints(), 
                bsplineGeometry.GetRearPoints(), 
                bsplineGeometry.GetFrontDerivative1(), 
                bsplineGeometry.GetRearDerivative1(), 
                ds, 
                targetPointState.getTime());
        }
        else{
            Debug.Log("デバックモードがOFFのためファイル出力しません.");
        }
    }

}

