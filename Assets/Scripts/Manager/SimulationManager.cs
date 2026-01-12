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

    public int simulationCount;
    public int totalSteps;
    public float tMax;

    [Header("シミュレーション設定")]
    public bool enableDebugOutput = true;
    public StreamWriter output;
    // public string outputFileName = "simulation_output.csv";
    public bool isPaused = false;
    public bool isInitialSimulationRunning = false;
    public bool enableOutputSimulationData = true;

    public bool enableGUIDetailMode = true;

    [Header("シミュレーションマネージャー")]
    public CalculationManager cal;

    public TeleportConfig teleport;

    public HitMarkerPart hitMarkerPart;



    public float[] time;      //時間
    public int n;

    private int i;//debugループカウンタ

    string fileName;

    void Start() {


        InitializeSimulatinManager();


    }

    void InitializeSimulatinManager()
    {

        totalSteps = 100001;
        simulationCount = 0;
        tMax = 500.0f;

        currentStep = 0;

        isSimulationRunning = false;  // シミュレーション実行状態
        isPaused = true;
        isInitialSimulationRunning = false;

        enableOutputSimulationData = true;
        enableDebugOutput = false;

        enableGUIDetailMode = false;

        
        teleport = new TeleportConfig();
        hitMarkerPart = new HitMarkerPart();

    }

    void ResetSimulatinManager()
    {
        currentStep = 0;

        isSimulationRunning = false;  // シミュレーション実行状態
        isInitialSimulationRunning = false;
        isPaused = true;
    }
    

    // Update is called once per frame
    void Update() {
        // キーボード入力処理
        HandleInput();

        HandleTeleportInput();

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


        // B
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

        // G
        // GUIの表示モード
        if (Input.GetKeyDown(KeyCode.G))
        {

            if(!enableGUIDetailMode)
            {
                enableGUIDetailMode = true;
                Debug.Log("GUI 詳細モード ON");
            }else
            {
                enableGUIDetailMode = false;
                Debug.Log("GUI 詳細モード OFF");
            }
        }

    }

    public void StartSimulation()
    {
        //  一番最初のシミュレーション開始
        if (!isInitialSimulationRunning)
        {
            
            simulationCount ++;

            // 出力ファイル設定
            string time = System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
            fileName = $"simulation_output_{simulationCount}_{time}.csv";
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
                "d1cs1ds11", "d2cs1ds11",
                "d1cs2ds21", "d2cs2ds22",
                "thetaT1", "thetaT2", "thetaP2d", "thetaP2", "thetaP2d - thetaP2",
                "d1", "d2", 
                "vehicle.w1", "vehicle.w2", "vehicle.w3", "steerInput"
            };

            string line1 = string.Join(",", header);

            output.WriteLine(line1);
            Debug.Log($"fileName:{fileName} 書き込み開始");

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

        Debug.Log("シミュレーションリセット中...");

        ResetSimulatinManager();

        hitMarkerPart.ResetHitMarker();



        if (cal != null)
        {
            cal.ResetCalculateManager();
            cal.FlushRemainingBuffer();

            Debug.Log($"fileName:{fileName} 書き込み終了");
        }

        Debug.Log("シミュレーションリセット完了");

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
        if (cal == null) return;

        // time
        float time = cal.vehicleRobotState.GetTime();  
        float time1 = cal.targetPointState.GetTime();  

        // m/s
        float u1_ms = cal.vehicleRobotState.GetU1();          // 車両速度
        float v1_ms = cal.targetPointState.GetV1();           // 目標点 前進速度

        TargetPointMode mode = cal.targetPointState.GetMode();

        // km/h
        float v1_kmh = v1_ms * 3.6f;
        float u1_kmh = u1_ms * 3.6f;

        // rad/s
        float v2_rads = cal.targetPointState.GetV2();

        // rad → deg
        float theta_deg = cal.targetPointState.GetTheta() * Mathf.Rad2Deg;
        float phi1_deg = cal.vehicleRobotState.GetPhi1() * Mathf.Rad2Deg;


        GUILayout.BeginArea(new Rect(10, 10, 260, 260));

        // GUILayout.Label($"current step: {currentStep}", GUI.skin.box);

        GUILayout.Label($"sim count: {simulationCount}", GUI.skin.box);

        GUILayout.Label($"time: {time}", GUI.skin.box);

        GUILayout.Label($"TargetPoint Mode: {mode}", GUI.skin.box);

        GUILayout.Label($"Vehicle Speed: {u1_kmh:F6} km/h", GUI.skin.box);

        GUILayout.Label($"stage: {teleport.GetCurrentStage().stageName}", GUI.skin.box);



        // GUILayout.Label($"time1 TP state: {time1}", GUI.skin.box);

        if(enableGUIDetailMode)
        {
            GUILayout.Label($"TargetPoint v1: {v1_ms:F6} m/s", GUI.skin.box);

            GUILayout.Label($"TargetPoint v1: {v1_kmh:F6} km/h", GUI.skin.box);

            GUILayout.Label($"TargetPoint v2: {v2_rads:F2} rad/s", GUI.skin.box);

            GUILayout.Label($"TargetPoint θ: {theta_deg:F1} deg", GUI.skin.box);

            GUILayout.Label($"Vehicle Speed: {u1_ms:F6} m/s", GUI.skin.box);

            GUILayout.Label($"Vehicle φ: {phi1_deg:F1} deg", GUI.skin.box);
        }

        GUILayout.EndArea();

        // ステージ移動候補表示
        if (isPaused && !isSimulationRunning)
        {
            GUILayout.BeginArea(new Rect(10, 280, 320, 200));
            GUILayout.Label("Teleport (paused only)");
            for (int i = 0; i < TeleportConfig.teleportPoints.Length; i++)
            {
                var p = TeleportConfig.teleportPoints[i];
                GUILayout.Label($"{i + 1}: {p.stageName} (x={p.x}, y={p.y}, theta={p.theta})");
            }
            GUILayout.EndArea();
        }
    }

    // ステージ移動の入力受付
    private void HandleTeleportInput()
    {
        if (!isPaused || isSimulationRunning) return;
        if (cal == null) return;

        if (Input.GetKeyDown(KeyCode.J))
        {

            ResetSimulatinManager();
            cal.FlushRemainingBuffer();

            Debug.Log($"fileName:{fileName} 書き込み終了");

            teleport.SetCurrentStage(0);

            cal.TeleportVehicleTo(TeleportConfig.teleportPoints[0]);
        }

        for (int i = 0; i < TeleportConfig.teleportPoints.Length && i < 11; i++)
        {
            KeyCode alpha = KeyCode.Alpha1 + i;
            KeyCode keypad = KeyCode.Keypad1 + i;

            if (Input.GetKeyDown(alpha) || Input.GetKeyDown(keypad))
            {
                ResetSimulatinManager();
                cal.FlushRemainingBuffer();

                Debug.Log($"fileName:{fileName} 書き込み終了");

                teleport.SetCurrentStage(i+1);

                cal.TeleportVehicleTo(TeleportConfig.teleportPoints[i+1]);
                break;
            }
        }
    }
}
