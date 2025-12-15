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
    public Transform TargetPoint1; 
    public Transform TargetPoint2; 
    public Renderer target1Renderer;
    public Renderer target2Renderer;


    public float scale = 1f;

    // trueなら0.01, falseならFixedDeltaTime
    public bool setFixedTimestepTo001 = true; // 最初の一回だけ 0.01 に変更

    private Vector3 targetPointPosition1;
    private Quaternion targetPointRotation1;

    private Vector3 targetPointPosition2;
    private Quaternion targetPointRotation2;

    private float targetPointSpeed;

    private Material mat1;
    private Material mat2;


    void Awake() {
        if (setFixedTimestepTo001) Time.fixedDeltaTime = 0.01f; // Project Settings > Time でも設定可
        if (target1Renderer != null) mat1 = target1Renderer.material;
        if (target2Renderer != null) mat2 = target2Renderer.material;

    }

    void Start() {
        if (sim == null) return;
        InitialPositions();

        UpdateColor();
    }

    // 初期配置
    void InitialPositions()
    {
        // ComputePositionAndRotation();

                // 現在の目標点がTp1の時、
        // Tp1は目標点の状態で移動
        // Tp2は後方車両、後輪間中点の座標に移動
        // // 前方目標点の初期設定
        float Tp1theta = calc.targetPointState.getTheta();
        float Tp1x = calc.targetPointState.getX();
        float Tp1y = calc.targetPointState.getY();

        float targetPoint1Deg  = -Tp1theta * Mathf.Rad2Deg;

        targetPointRotation1  = Quaternion.Euler(0f, targetPoint1Deg, 0f);
        targetPointPosition1  = new Vector3(Tp1x, 0.5f, Tp1y)*scale;;

        // 後方目標点の初期設定
        float Tp2theta = calc.vehicleRobotState.GetTheta3(); // theta3に設定
        // 後方車両 後輪間中点の座標取得
        float Tp2x = calc.vehicleRobotState.GetMidpointBetweenRearWheelsOfSV().x;
        float Tp2y = calc.vehicleRobotState.GetMidpointBetweenRearWheelsOfSV().y;

        float targetPoint2Deg  = -Tp2theta * Mathf.Rad2Deg;

        targetPointRotation2  = Quaternion.Euler(0f, targetPoint2Deg, 0f);
        targetPointPosition2  = new Vector3(Tp2x, 0.5f, Tp2y)*scale;

        UpdatePositions();
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

        // UpdateColor();

        // ここで色変化をチェック
        if (_current != _prev)
        {
            UpdateColor();
        }
    }


    // 毎フレーム
    void ComputePositionAndRotation()
    {
        var tp = calc.targetPointState.GetCurrentTargetPoint();

        // Debug.Log($"calc.targetPointState.GetCurrentTargetPoint():{calc.targetPointState.GetCurrentTargetPoint()}");


        if(tp == CurrentTargetPoint.Tp1)
        {
            // Tp1 ← state の位置
            float x = calc.targetPointState.getX();
            float y = calc.targetPointState.getY();
            float th = calc.targetPointState.getTheta();

            targetPointPosition1 = new Vector3(x, 0.5f, y) * scale;
            targetPointRotation1 = Quaternion.Euler(0, -th * Mathf.Rad2Deg, 0);

            // Tp2 ← 後方車両の rear midpoint
            var rear = calc.vehicleRobotState.GetMidpointBetweenRearWheelsOfSV();
            float th2 = calc.vehicleRobotState.GetTheta3();

            targetPointPosition2 = new Vector3(rear.x, 0.5f, rear.y) * scale;
            targetPointRotation2 = Quaternion.Euler(0, -th2 * Mathf.Rad2Deg, 0);
        }
        else // Tp2 が現在のターゲット
        {
            // Tp2 ← state の位置
            float x = calc.targetPointState.getX();
            float y = calc.targetPointState.getY();
            float th = calc.targetPointState.getTheta();

            // Debug.Log($"x,y:{x}, {y}");

            targetPointPosition2 = new Vector3(x, 0.5f, y) * scale;
            targetPointRotation2 = Quaternion.Euler(0, -th * Mathf.Rad2Deg, 0);

            // Tp1 ← 前方車両 front midpoint
            var front = calc.vehicleRobotState.GetMidpointBetweenFrontWheelsOfFV();
            float th1 = calc.vehicleRobotState.GetTheta1();

            // Debug.Log($"front.x,front.y:{front.x}, {front.y}");

            targetPointPosition1 = new Vector3(front.x, 0.5f, front.y) * scale;
            targetPointRotation1 = Quaternion.Euler(0, -th1 * Mathf.Rad2Deg, 0);
        }
    }

    void UpdatePositions()
    {
        // 目標点の移動
        TargetPoint1.transform.SetPositionAndRotation(targetPointPosition1, targetPointRotation1);
        TargetPoint2.transform.SetPositionAndRotation(targetPointPosition2, targetPointRotation2);
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
        if (mat1 == null) return;
        if (mat2 == null) return;

        TargetPointMode _currentMode = calc.targetPointState.GetMode();

        switch (_currentMode)
        {
            case TargetPointMode.Forward:
                mat1.SetColor("_BaseColor", Color.green);
                mat2.SetColor("_BaseColor", Color.green);
                break;
            case TargetPointMode.Parking:
                mat1.SetColor("_BaseColor", Color.yellow);
                mat2.SetColor("_BaseColor", Color.yellow);

                break;
            case TargetPointMode.Back:
                mat1.SetColor("_BaseColor", Color.blue);
                mat2.SetColor("_BaseColor", Color.blue);
                break;
        }
    }

    public float GetCurrentTargetPointSpeed() { 
        return targetPointSpeed; 
    }
    public Vector3 GetCurrentPosition() { 
        return targetPointPosition1;
    }
}