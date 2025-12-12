using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConnectionRenderer : MonoBehaviour
{
    [Header("Simulation")]
    public SimulationManager sim;

    [Header("Calculation")]
    public CalculationManager calc;

    [Header("ConnectionPoint")]
    public Transform ConnectionPoint1; 
    public Transform ConnectionPoint2; 
    public Renderer connectonRenderer;

    public float scale = 1f;

    // trueなら0.01, falseならFixedDeltaTime
    public bool setFixedTimestepTo001 = true; // 最初の一回だけ 0.01 に変更

    private Vector3 connectionPoint1Position;
    private Quaternion connectionPoint1Rotation;

    private Vector3 connectionPoint2Position;
    private Quaternion connectionPoint2Rotation;

    private Material mat;

    void Awake() {
        if (setFixedTimestepTo001) Time.fixedDeltaTime = 0.01f; // Project Settings > Time でも設定可
        if (connectonRenderer != null) mat = connectonRenderer.material;
    }

    void Start() {
        if (sim == null) return;
        InitialPositions();
    }

    // 初期配置
    void InitialPositions()
    {
        // float theta = calc.vehicleRobotState.getTheta();
        // float x = calc.vehicleRobotState.getX();
        // float y = calc.vehicleRobotState.getY();

        // float targetPointDeg  = -theta * Mathf.Rad2Deg;

        // targetPointRotation  = Quaternion.Euler(0f, targetPointDeg, 0f);

        // targetPointPosition  = new Vector3(x, 0.5f, y)*scale;;
        // // 反映
        // TargetPoint.SetPositionAndRotation(targetPointPosition, targetPointRotation);

        ComputePositionAndRotation();
        UpdatePositions();
    }


    void FixedUpdate() 
    {
        if (sim == null) return;
        // if (sim.isSimulationRunning)
        // {
        //     ComputePositionAndRotation();
        // }

            ComputePositionAndRotation();

    }

    void Update()
    {
    }

    // 毎フレーム
    void ComputePositionAndRotation()
    {
        float x1 = calc.vehicleRobotState.GetX1();
        float y1 = calc.vehicleRobotState.GetY1();
        float theta2 = calc.vehicleRobotState.GetTheta2();
        
        float x2 = calc.vehicleRobotState.GetX2();
        float y2 = calc.vehicleRobotState.GetY2();
        float theta3 = calc.vehicleRobotState.GetTheta3();

        float connectionPoint1Deg  = -theta2*Mathf.Rad2Deg;
        float connectionPoint2Deg  = -theta3*Mathf.Rad2Deg;

        connectionPoint1Position  = new Vector3(x1, 0.5f, y1) * scale;
        connectionPoint1Rotation  = Quaternion.Euler(0f, connectionPoint1Deg, 0f);

        connectionPoint2Position  = new Vector3(x2, 0.5f, y2) * scale;
        connectionPoint2Rotation  = Quaternion.Euler(0f, connectionPoint2Deg, 0f);
    }


    void UpdatePositions()
    {
        // 第1連結点の移動
        ConnectionPoint1.transform.SetPositionAndRotation(connectionPoint1Position, connectionPoint1Rotation);

        // 第2連結点の移動
        ConnectionPoint2.transform.SetPositionAndRotation(connectionPoint2Position, connectionPoint2Rotation);
    }
    // Update is called once per frame
    void LateUpdate() {
        // if (sim.isSimulationRunning)
        // {
        //     UpdatePositions();
        // }

        UpdatePositions();

    }

    // public float GetCurrentTargetPointSpeed() { 
    //     return targetPointSpeed; 
    // }
    // public Vector3 GetCurrentPosition() { 
    //     return targetPointPosition;
    // }
}