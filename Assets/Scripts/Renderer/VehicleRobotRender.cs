using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VehicleRobotRenderer : MonoBehaviour
{
    [Header("Simulation")]
    public SimulationManager sim;

    [Header("Calculation")]
    public CalculationManager calc;

    [Header("FirstVehicle")]
    public Transform FirstVehicle; 
    public Transform steerFROfFV, steerFLOfFV; 
    public Transform wheelFROfFV, wheelFLOfFV;   // タイヤの見た目（メッシュ）
    public Transform wheelRROfFV, wheelRLOfFV;   // タイヤの見た目（メッシュ）

    [Header("SecondVehicle")]
    public Transform SecondVehicle; 
    public Transform steerRROfSV, steerRLOfSV; 
    public Transform wheelFROfSV, wheelFLOfSV;   // タイヤの見た目（メッシュ）
    public Transform wheelRROfSV, wheelRLOfSV;   // タイヤの見た目（メッシュ）

    [Header("TruckBed")]
    public Transform TruckBed;

    public Renderer vehicleRenderer;
    private Material mat;

    // trueなら0.01, falseならFixedDeltaTime
    public bool setFixedTimestepTo001 = true; // 最初の一回だけ 0.01 に変更

    // 先頭車両
    private Vector3 firstVehiclePosition;
    private Quaternion firstVehicleRotation;

    // 後方車両
    private Vector3 secondVehiclePosition;
    private Quaternion secondVehicleRotation;

    // 荷台
    private Vector3 truckBedPosition;
    private Quaternion truckBedRotation;

    //　ステア角
    private Quaternion steerRotationOfFV;
    private Quaternion steerRotationOfSV;

    //　ホイール角
    private Quaternion wheelRotationOfFV;
    private Quaternion wheelRotationOfSV;

    // 第一車両のタイヤ 回転角　初期変数
    private Quaternion steerFLOfFVLocalRotInitial, steerFROfFVLocalRotInitial; 
    private Quaternion wheelFROfFVLocalRotInitial, wheelFLOfFVLocalRotInitial; // 前輪左右
    private Quaternion wheelRROfFVLocalRotInitial, wheelRLOfFVLocalRotInitial; // 後輪左右

    // 第二車両のタイヤ 初期変数
    private Quaternion steerRLOfSVLocalRotInitial, steerRROfSVLocalRotInitial;
    private Quaternion wheelFROfSVLocalRotInitial, wheelFLOfSVLocalRotInitial; // 前輪左右
    private Quaternion wheelRROfSVLocalRotInitial, wheelRLOfSVLocalRotInitial; // 後輪左右

    private float firstVehicleSpeed;
    private float secondVehicleSpeed;
    private float truckBedSpeed;

    private float wheelAngleOfFV;
    private float wheelAngleOfSV;

    private float wheelRadius; // [m]

    public float scale;

    void Awake() {
        if (setFixedTimestepTo001) Time.fixedDeltaTime = 0.01f; // Project Settings > Time でも設定可
        if (vehicleRenderer != null) mat = vehicleRenderer.material;

        // 第一車両
        steerFLOfFVLocalRotInitial = steerFLOfFV.localRotation;
        steerFROfFVLocalRotInitial = steerFROfFV.localRotation;

        wheelFROfFVLocalRotInitial = wheelFROfFV.localRotation;
        wheelFLOfFVLocalRotInitial = wheelFLOfFV.localRotation;

        wheelRROfFVLocalRotInitial = wheelRROfFV.localRotation;
        wheelRLOfFVLocalRotInitial = wheelRLOfFV.localRotation;

        // 第二車両
        steerRLOfSVLocalRotInitial = steerRLOfSV.localRotation;
        steerRROfSVLocalRotInitial = steerRROfSV.localRotation;

        wheelFROfSVLocalRotInitial = wheelFROfSV.localRotation;
        wheelFLOfSVLocalRotInitial = wheelFLOfSV.localRotation;

        wheelRROfSVLocalRotInitial = wheelRROfSV.localRotation;
        wheelRLOfSVLocalRotInitial = wheelRLOfSV.localRotation;
    }

    void Start() {
        if (sim == null) return;

        scale = 1f;
        wheelAngleOfFV = 0f;
        wheelAngleOfSV = 0f;
        wheelRadius = 0.125f; // [m]

        InitialPositions();

        // transform.localScale = new Vector3(2f, 1f, 3f);
    }

    // 初期配置
    void InitialPositions()
    {
        ComputePositionAndRotation();
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
    }


    // 毎フレーム
    void ComputePositionAndRotation()
    {
        float x1 = calc.vehicleRobotState.GetX1();
        float y1 = calc.vehicleRobotState.GetY1();
        float x2 = calc.vehicleRobotState.GetX2();
        float y2 = calc.vehicleRobotState.GetY2();
        float x3 = calc.vehicleRobotState.GetX3();
        float y3 = calc.vehicleRobotState.GetY3();
        float phi1 = calc.vehicleRobotState.GetPhi1();
        float phi2 = calc.vehicleRobotState.GetPhi2();
        float theta1 = calc.vehicleRobotState.GetTheta1();
        float theta2 = calc.vehicleRobotState.GetTheta2();
        float theta3 = calc.vehicleRobotState.GetTheta3();

        firstVehicleSpeed  = calc.vehicleRobotState.GetU1();
        secondVehicleSpeed = calc.vehicleRobotState.GetU4(); // ここは本当に u4 ？（u1,u2,u3 だけなら見直し推奨）

        // 先頭車両
        Vector3 offsetLocalForFV = new Vector3(0.25f, 0f, 0f);

        float firstVehicleDeg  = -theta1 * Mathf.Rad2Deg;
        firstVehicleRotation  = Quaternion.Euler(0f, firstVehicleDeg, 0f);
        Vector3 fvBase = new Vector3(x1, 0f, y1) * scale;
        firstVehiclePosition  = fvBase + firstVehicleRotation  * (offsetLocalForFV * scale);
        

        // 後方車両
        Vector3 offsetLocalForSV = new Vector3(-0.25f, 0f, 0f);

        float secondVehicleDeg = -theta3 * Mathf.Rad2Deg;
        secondVehicleRotation = Quaternion.Euler(0f, secondVehicleDeg, 0f);
        Vector3 svBase = new Vector3(x2, 0f, y2) * scale;
        secondVehiclePosition = svBase + secondVehicleRotation * (offsetLocalForSV * scale);
        
        // 荷台
        float truckBedDeg      = -theta2 * Mathf.Rad2Deg;
        truckBedRotation      = Quaternion.Euler(0f, truckBedDeg, 0f);
        truckBedPosition = new Vector3(x3, 0f, y3) * scale;

        // ステア
        float steerDegOfFV = -phi1 * Mathf.Rad2Deg;
        steerRotationOfFV = Quaternion.Euler(0f, steerDegOfFV, 0f);
        float steerDegOfSV = -phi2 * Mathf.Rad2Deg;
        steerRotationOfSV = Quaternion.Euler(0f, steerDegOfSV, 0f);

        // ホイール
        float wheelOmegaOfFV = (firstVehicleSpeed  / wheelRadius) * Time.fixedDeltaTime;
        wheelAngleOfFV += -wheelOmegaOfFV * Mathf.Rad2Deg;
        wheelRotationOfFV = Quaternion.Euler( 0f, wheelAngleOfFV ,0f); 
        
        float wheelOmegaOfSV = (secondVehicleSpeed / wheelRadius) * Time.fixedDeltaTime;
        wheelAngleOfSV += -wheelOmegaOfSV * Mathf.Rad2Deg;
        wheelRotationOfSV = Quaternion.Euler( 0f, wheelAngleOfSV ,0f); //右手から左手に変換
    }


    void UpdatePositions()
    {

        // 第1車両オブジェクトの移動 firstVehicle
        FirstVehicle.transform.SetPositionAndRotation(firstVehiclePosition, firstVehicleRotation);

        // 先頭車両の前輪ステア
        steerFROfFV.localRotation = steerFROfFVLocalRotInitial * steerRotationOfFV;
        steerFLOfFV.localRotation = steerFLOfFVLocalRotInitial * steerRotationOfFV;
        // 先頭車両の前輪ホイール
        wheelFLOfFV.localRotation = wheelFLOfFVLocalRotInitial * wheelRotationOfFV;
        wheelFROfFV.localRotation = wheelFROfFVLocalRotInitial * wheelRotationOfFV;
        // 先頭車両の後輪ホイール
        wheelRLOfFV.localRotation = wheelRLOfFVLocalRotInitial * wheelRotationOfFV;
        wheelRROfFV.localRotation = wheelRROfFVLocalRotInitial * wheelRotationOfFV;

        // 荷台オブジェクトの移動 truckBed
        TruckBed.transform.SetPositionAndRotation(truckBedPosition, truckBedRotation);

        // 第二車両オブジェクトの移動 secondVehicle
        SecondVehicle.transform.SetPositionAndRotation(secondVehiclePosition, secondVehicleRotation);
        // 後方車両の前輪ステア
        steerRROfSV.localRotation = steerRROfSVLocalRotInitial * steerRotationOfSV;
        steerRLOfSV.localRotation = steerRLOfSVLocalRotInitial * steerRotationOfSV;
        // 後方車両の前輪ホイール
        wheelFLOfSV.localRotation = wheelFLOfSVLocalRotInitial * wheelRotationOfSV;
        wheelFROfSV.localRotation = wheelFROfSVLocalRotInitial * wheelRotationOfSV;
        // 後方車両の後輪ホイール
        wheelRLOfSV.localRotation = wheelRLOfSVLocalRotInitial * wheelRotationOfSV;
        wheelRROfSV.localRotation = wheelRROfSVLocalRotInitial * wheelRotationOfSV;
    }
    // Update is called once per frame
    void LateUpdate() {
        if (sim.isSimulationRunning)
        {
            UpdatePositions();
        }
    }

    // public float GetCurrentTargetPointSpeed() { 
    //     return targetPointSpeed; 
    // }
    // public Vector3 GetCurrentPosition() { 
    //     return targetPointPosition;
    // }
}