using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using System.IO;
using System;
using System.Text;            // UTF8Encoding ← これが不足でCS0246

public class SimulationManager : MonoBehaviour {
    [Header("シミュレーション制御")]
    public bool isSimulationRunning = false;  // シミュレーション実行状態
    public int currentStep;
    public int totalSteps;
    public float tMax;

    [Header("シミュレーション設定")]
    public bool enableDebugOutput = true;
    public StreamWriter output;
    // public string outputFileName = "simulation_output.csv";
    public bool isPaused = false;
    public bool isInitialSimulationRunning = false;
    public bool enableOutputSimulationData = true;

    [Header("シミュレーションマネージャー")]
    public CalculationManager cal;


    public float[] time;      //時間
    public Vector3[] points;   // 第一関節 位置座標ベクトル（x,y→X投影）
    public int n;

    private int i;//debugループカウンタ

    void Start() {

        totalSteps = 100001;
        currentStep = 0;

        points = new Vector3[totalSteps];

        tMax = 500.0f;

        isSimulationRunning = false;  // シミュレーション実行状態
        enableDebugOutput = false;
        isPaused = true;
        isInitialSimulationRunning = false;
        enableOutputSimulationData = true;

        // 出力ファイル設定
        string time = System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        string fileName = $"simulation_output_{time}.csv";
        output = new StreamWriter(@fileName, false, Encoding.GetEncoding("Shift_JIS"));

        string[] header = {
            "time",
            "s",
            "Tp.x", "Tp.y", 
            "theta",
            "Tp.v1", "Tp.v2",
            "vehicle.x1", "vehicle.y1", 
            "vehicle.phi1", "vehicle.theta1", 
            "vehicle.phi2", "vehicle.theta2", "vehicle.theta3",
            "vehicle.x2", "vehicle.y2", 
            "vehicle.u1", "vehicle.u2", "vehicle.u3", "vehicle.u4",
            "u1Index", "u2Index",
            "u1Float", "u2Float",
            "rx1", "ry1",
            "d1rx1du11", "d1ry1du11",
            "d2rx1du12", "d2ry1du12",
            "d3rx1du13", "d3ry1du13",
            "d4rx1du14", "d4ry1du14",
            "rx2", "ry2",
            "cs1", "cs2",
            "thetaT1", "thetaT2", "thetaP2d",
            "d1", 
            "vehicle.w1", "vehicle.w2", "vehicle.w3"
        };

        string line1 = string.Join(",", header);

        output.WriteLine(line1);
    }

    // Update is called once per frame
    void Update() {
        // キーボード入力処理
        HandleInput();

        if(currentStep >= totalSteps)
        {
            EndSimulation();
        }
    }
    
    private void HandleInput()
    {
        // Sキーでシミュレーション開始 or 再スタート
        if (Input.GetKeyDown(KeyCode.S))
        {
            StartSimulation();
        }
        
        // Tキーでシミュレーション停止
        if (Input.GetKeyDown(KeyCode.T))
        {
            StopSimulation();
        }

        // Qキーでシミュレーション終了
        if (Input.GetKeyDown(KeyCode.Q))
        {
            EndSimulation();
        }

        // Rキーでシミュレーション　リセット
        if (Input.GetKeyDown(KeyCode.R))
        {
            ResetSimulation();
        }

        // Oキーで書き込みモードON
        if (Input.GetKeyDown(KeyCode.O))
        {
            if(!enableOutputSimulationData)
            {
                enableOutputSimulationData = true;
                Debug.Log("書き込みモード ON");
            }else
            {
                enableOutputSimulationData = false;
                Debug.Log("書き込みモード OFF");
            }
        }

        // Dキーで書き込みモードON
        if (Input.GetKeyDown(KeyCode.D))
        {
            if(!enableDebugOutput)
            {
                enableDebugOutput = true;
                Debug.Log("デバックモード ON");
            }else
            {
                enableDebugOutput = false;
                Debug.Log("デバックモード OFF");
            }
        }

        if (Input.GetKeyDown(KeyCode.B))
        {
            if(isPaused)
            {
                cal.OutputDebug();
            }else
            {
                Debug.Log("Bspline曲線を出力するにはシミュレーションを一時停止してください.(T)");
            }
        }

        // B
    }

    public void StartSimulation()
    {
        //  一番最初のシミュレーション開始
        if (!isInitialSimulationRunning)
        {
            isInitialSimulationRunning = true;
            isPaused = false;
            isSimulationRunning = true;
            Debug.Log("シミュレーション開始");
        }

        //  シミュレーション開始後に再スタートしたいとき
        if (isInitialSimulationRunning && isPaused)
        {
            isPaused = false;
            isSimulationRunning = true;
            Debug.Log("シミュレーション 再開");
        }
    }

    public void StopSimulation()
    {
        if (isSimulationRunning)
        {
            isSimulationRunning = false; 
            isPaused = true;
            Debug.Log("シミュレーション停止");
        }
        else
        {
            Debug.Log("シミュレーションは既に停止中です");
        }
    }

    public void EndSimulation()
    {
        isSimulationRunning = false;
        isInitialSimulationRunning = false;
        currentStep = 0;


        if (cal != null)
        {
            cal.FlushRemainingBuffer();
        }

        SaveResults();


        Debug.Log("シミュレーション終了");

        return;
    }

    public void ResetSimulation()
    {
        isSimulationRunning = false;
        isInitialSimulationRunning = false;
        isPaused = false;

        currentStep = 0;

        // 数値計算シミュレーションのリセット(CalculationManager.cs1)
        cal.InitializeCalculateSimulation();


        Debug.Log("シミュレーションリセット");

        return;
    }

    public void SaveResults()
    {
        if (!enableOutputSimulationData)
        {
            Debug.Log("書き込みモードがOFFのため、ファイル出力をスキップします");
            return;
        }

        // 保存先パス
        string time = System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        string fileName = $"Bspline_output_{time}.csv";
        string path = Path.Combine(Application.persistentDataPath,fileName);

        try
        {

            output.Close();
            // Debug.Log($"結果を保存しました: {path}");
            Debug.Log($"結果を保存しました");
        }
        catch (Exception ex)
        {
            Debug.LogError($"SaveResults failed: {ex.GetType().Name}: {ex.Message}\nPath: {path}");
        }
    }
    
    private void RunSimulation()
    {
        // ここに実際のシミュレーション処理を実装
        // 例：車両の移動、ベジエ曲線に沿った移動など
        
        // デバッグ用（実際の処理に置き換えてください）
    }

    private string F(float v) => v.ToString("F6", CultureInfo.InvariantCulture);

    void OnGUI()
    {
        if (this == null) return;

        // m/s
        float v1_ms = cal.vehicleRobotState.GetU1();          // 車両速度
        float u1_ms = cal.targetPointState.getV1();           // 目標点 前進速度

        TargetPointMode mode = cal.targetPointState.GetMode();

        // km/h
        float v1_kmh = v1_ms * 3.6f;
        float u1_kmh = u1_ms * 3.6f;

        // rad/s
        float v2_rads = cal.targetPointState.getV2();

        // rad → deg
        float theta_deg = cal.targetPointState.getTheta() * Mathf.Rad2Deg;
        float phi1_deg = cal.vehicleRobotState.GetPhi1() * Mathf.Rad2Deg;


        GUILayout.BeginArea(new Rect(10, 10, 260, 260));

        GUILayout.Label($"TargetPoint Mode: {mode}", GUI.skin.box);

        GUILayout.Label($"TargetPoint u1: {u1_kmh:F6} km/h", GUI.skin.box);

        GUILayout.Label($"TargetPoint u1: {u1_ms:F6} m/s", GUI.skin.box);

        GUILayout.Label($"TargetPoint v2: {v2_rads:F2} rad/s", GUI.skin.box);

        GUILayout.Label($"TargetPoint θ: {theta_deg:F1} deg", GUI.skin.box);

        GUILayout.Label($"Vehicle Speed: {v1_ms:F6} m/s", GUI.skin.box);

        GUILayout.Label($"Vehicle φ: {phi1_deg:F1} deg", GUI.skin.box);


        GUILayout.EndArea();
    }

}
