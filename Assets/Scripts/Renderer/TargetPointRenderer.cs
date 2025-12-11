using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TargetPointRenderer : MonoBehaviour
{
    [Header("Simulation")]
    public SimulationManager sim;

    [Header("Calculation")]
    public CalculationManager calc;

    [Header("TargetPoint")]
    public Transform TargetPoint; 
    public Renderer targetRenderer;

    public float scale = 1f;

    // trueなら0.01, falseならFixedDeltaTime
    public bool setFixedTimestepTo001 = true; // 最初の一回だけ 0.01 に変更

    private Vector3 targetPointPosition;
    private Quaternion targetPointRotation;
    private float targetPointSpeed;

    private Material mat;

    void Awake() {
        if (setFixedTimestepTo001) Time.fixedDeltaTime = 0.01f; // Project Settings > Time でも設定可
        if (targetRenderer != null) mat = targetRenderer.material;
    }

    void Start() {
        if (sim == null) return;
        InitialPositions();

        calc.targetPointState.SetCurrentMode(TargetPointMode.Stop);
        calc.targetPointState.SetPrevMode(TargetPointMode.Stop);

        UpdateColor();
    }

    // 初期配置
    void InitialPositions()
    {
        float theta = calc.targetPointState.getTheta();
        float x = calc.targetPointState.getX();
        float y = calc.targetPointState.getY();

        float targetPointDeg  = -theta * Mathf.Rad2Deg;

        targetPointRotation  = Quaternion.Euler(0f, targetPointDeg, 0f);

        targetPointPosition  = new Vector3(x, 0.5f, y)*scale;;
        // 反映
        TargetPoint.SetPositionAndRotation(targetPointPosition, targetPointRotation);
    }


    void FixedUpdate() 
    {
        if (sim == null) return;
        if (sim.isSimulationRunning)
        {
            ComputePositionAndRotation();
        }
    }

    void Update()
    {
        if (sim == null || !sim.isSimulationRunning) return;

        TargetPointMode _prev = calc.targetPointState.GetPrevMode();
        TargetPointMode _current = calc.targetPointState.GetMode();


        // ここで色変化をチェック
        if (_current != _prev)
        {
            UpdateColor();
            calc.targetPointState.SetPrevMode(calc.targetPointState.GetMode()); // ←追加
        }
    }


    // 毎フレーム
    void ComputePositionAndRotation()
    {
        float x = calc.targetPointState.getX();
        float y = calc.targetPointState.getY();
        float theta = calc.targetPointState.getTheta();
        float v1 = calc.targetPointState.getV1();
        float v2 = calc.targetPointState.getV2();

        targetPointSpeed  = v1;

        float targetPointDeg  = -theta*Mathf.Rad2Deg;

        targetPointRotation  = Quaternion.Euler(0f, targetPointDeg, 0f);
        targetPointPosition  = new Vector3(x, 0.5f, y) * scale;
    }


    void UpdatePositions()
    {
        // 目標点の移動
        TargetPoint.transform.SetPositionAndRotation(targetPointPosition, targetPointRotation);
    }
    // Update is called once per frame
    void LateUpdate() {
        if (sim.isSimulationRunning)
        {
            UpdatePositions();
        }
    }

    void UpdateColor()
    {
        if (mat == null) return;

        TargetPointMode _current = calc.targetPointState.GetMode();
        Debug.Log($"_current:{_current}");


        switch (_current)
        {
            case TargetPointMode.Forward:
                mat.SetColor("_BaseColor", Color.green);
                break;
            case TargetPointMode.Stop:
                mat.SetColor("_BaseColor", Color.yellow);
                break;
            case TargetPointMode.Back:
                mat.SetColor("_BaseColor", Color.blue);
                break;
        }
    }

    public float GetCurrentTargetPointSpeed() { 
        return targetPointSpeed; 
    }
    public Vector3 GetCurrentPosition() { 
        return targetPointPosition;
    }
}