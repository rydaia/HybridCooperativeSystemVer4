
using System;
using UnityEngine;
using Unity.Mathematics;

public class VehicleRobotStateData : State
{
    // 親クラスのStateに状態追加
    public float x1;
    public float y1;
    public float theta1;
    public float theta2;
    public float theta3;
    public float phi1;
    public float phi2;
    public float u1;     // 前進速度
    public float u2;     // 第１車両 ステア角速度
    public float u3;     // 第２車両 ステア角速度
    public float u4;     // 第２車両 ステア角速度

    public float x2;
    public float y2;
    public float x3;
    public float y3;
}

public class VehicleRobotState
{

    // フィールド
    // TargetPointStateDataクラスのインスタンス
    public VehicleRobotStateData current;

    private VehicleRobotDynamics dynamics;
    private VehicleKinematics vehicle;
    private RungeKutta runge;
    private VehicleParameters p;


    // DIM = 微分方程式の数（x1, y1, phi1, theta1, phi2, theta2, theta3) の7つ
    private int DIM = 7;

    // コントラスタ
    // クラスが new された瞬間に 最初に一度だけ実行される特別な関数
    public VehicleRobotState(VehicleRobotDynamics dyn, VehicleParameters p) 
    {
        // ここでTargetPointStateDataクラスのインスタンス生成
        current = new VehicleRobotStateData();

        this.dynamics = dyn;
        this.p = p;
    }

    public void Initialize(VehicleKinematics v)
    {

        this.vehicle = v;

        float l1 = p.GetL1();
        float l2 = p.GetL2();

        Debug.Log($"l1:{l1}");

        SetTime(0.0f);
        SetX1(-l1);
        SetY1(0.0f);
        SetPhi1(0.0f);
        SetTheta1(0.0f);
        SetPhi2(0.0f);
        SetTheta2(0.0f);
        SetTheta3(0.0f);
        SetX2(-(l1+l2));
        SetY2(0.0f);
        SetX2(-(l1 + l2/2f));
        SetY2(0.0f);

        Debug.Log($"Initital vehicle.x1, vehicle.y1:{GetX1()}, {GetY1()}");


        runge = new RungeKutta(DIM, new Func<float[], float>[] {
            (x) => dynamics.f0(current),
            (x) => dynamics.f1(current),
            (x) => dynamics.f2(current),
            (x) => dynamics.f3(current),
            (x) => dynamics.f4(current),
            (x) => dynamics.f5(current),
            (x) => dynamics.f6(current),
            (x) => dynamics.f7(current),
        });

        runge.x_old[0] = current.t;
        runge.x_old[1] = current.x1;
        runge.x_old[2] = current.y1;
        runge.x_old[3] = current.phi1;
        runge.x_old[4] = current.theta1;
        runge.x_old[5] = current.phi2;
        runge.x_old[6] = current.theta2;
        runge.x_old[7] = current.theta3;
    }

    public void updateVehicleRobotState(float dt)
    {

        // u1, u2, u3のセット
        SetU1(vehicle.GetU1());
        SetU2(vehicle.GetU2());
        SetU3(vehicle.GetU3());
        SetU4(vehicle.GetU4());

        runge.UpdateStateRungeKutta(dt);

        float maxPhi1 = 0.610865f; // 35度
        float minPhi1 = -0.610865f; // 35度

        float maxPhi2 = 0.610865f; // 35度
        float minPhi2 = -0.610865f; // 35度        float maxPhi1 = 0.610865f; // 35度

        

        // 結果を State に反映する
        SetTime(runge.x_new[0]);
        SetX1(runge.x_new[1]); 
        SetY1(runge.x_new[2]); 
        // SetPhi1(ClampSteer(runge.x_new[3]));
        SetPhi1(ClampSteer(runge.x_new[3]));

        SetTheta1(runge.x_new[4]);
        SetPhi2(ClampSteer(runge.x_new[5]));
        // SetPhi2(ClampSteer(runge.x_new[5]));
        SetTheta2(runge.x_new[6]);
        SetTheta3(runge.x_new[7]);

        float x1 = GetX1();
        float y1 = GetY1();

        float l2 = p.GetL2();
        float theta2 = GetTheta2();

        float x2 = x1 - l2 * Mathf.Cos(theta2);
        float y2 = y1 - l2 * Mathf.Sin(theta2);

        SetX2(x2); 
        SetY2(y2); 

        float x3 = x1 - (l2 / 2f) * Mathf.Cos(theta2);
        float y3 = y1 - (l2 / 2f) * Mathf.Sin(theta2);

        SetX3(x3); 
        SetY3(y3); 

        runge.CommitStep();
    }

    float ClampSteer(float phi)
    {
        const float MAX = 35f * Mathf.Deg2Rad;  
        return Mathf.Clamp(phi, -MAX, +MAX);
    }


    // private float NormalizeAngle(float angle)
    // {
    //     return math.atan2(math.sin(angle), math.cos(angle));
    // }

    // Setter
    public void SetTime(float v) { current.t = v; }
    public void SetX1(float v) { current.x1 = v; }
    public void SetY1(float v) { current.y1 = v; }
    public void SetPhi1(float v) { current.phi1 = v; }
    public void SetTheta1(float v) { current.theta1 = v; }
    public void SetPhi2(float v) { current.phi2 = v; }
    public void SetTheta2(float v) { current.theta2 = v; }
    public void SetTheta3(float v) { current.theta3 = v; }

    public void SetX2(float v) { current.x2 = v; }
    public void SetY2(float v) { current.y2 = v; }

    public void SetX3(float v) { current.x3 = v; }
    public void SetY3(float v) { current.y3 = v; }

    public void SetU1(float v) { current.u1 = v; }
    public void SetU2(float v) { current.u2 = v; }
    public void SetU3(float v) { current.u3 = v; }
    public void SetU4(float v) { current.u4 = v; }


    // Getter
    public float GetTime() => current.t;
    public float GetX1() => current.x1;
    public float GetY1() => current.y1;
    public float GetPhi1() => current.phi1;
    public float GetTheta1() => current.theta1;
    public float GetPhi2() => current.phi2;
    public float GetTheta2() => current.theta2;
    public float GetTheta3() => current.theta3;

    public float GetX2() => current.x2;
    public float GetY2() => current.y2;

    public float GetX3() => current.x3;
    public float GetY3() => current.y3;

    public float GetU1() => current.u1;
    public float GetU2() => current.u2;
    public float GetU3() => current.u3;
    public float GetU4() => current.u4;

    // 先頭車両 前輪間中点 Tp1
    public Vector2 GetMidpointBetweenFrontWheelsOfFV()
    {
        Vector2 ret;

        Vector2 pos = GetMidpointBetweenRearWheelsOfFV();
        float l1 = p.GetL1();
        float theta1 =  GetTheta1();

        ret.x = pos.x + l1 * Mathf.Cos(theta1);
        ret.y = pos.y + l1 * Mathf.Sin(theta1);

        return ret;
    }


    // 先頭車両 後輪間中点 (x1, y1)
    public Vector2 GetMidpointBetweenRearWheelsOfFV()
    {
        Vector2 pos;

        pos.x = GetX1();
        pos.y = GetY1();

        return pos;
    }

    // 後方車両 前輪間中点 (x2, y2)
    public Vector2 GetMidpointBetweenFrontWheelsOfSV()
    {
        Vector2 pos;

        pos.x = GetX2();
        pos.y = GetY2();

        return pos;
    }


    // 後方車両 後輪間中点 Tp2
    public Vector2 GetMidpointBetweenRearWheelsOfSV()
    {
        Vector2 ret;

        Vector2 pos = GetMidpointBetweenFrontWheelsOfSV();
        float l3 = p.GetL3();
        float theta3 =  GetTheta3();

        ret.x = pos.x - l3 * Mathf.Cos(theta3);
        ret.y = pos.y - l3 * Mathf.Sin(theta3);

        return ret;
    }
}
