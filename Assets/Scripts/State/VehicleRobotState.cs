
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
}

public class VehicleRobotState
{

    // フィールド
    // TargetPointStateDataクラスのインスタンス
    public VehicleRobotStateData current;

    private VehicleRobotDynamics dynamics;
    private VehicleKinematics vehicle;
    private RungeKutta runge;

    // DIM = 微分方程式の数（x1, y1, phi1, theta1, phi2, theta2, theta3) の7つ
    private int DIM = 7;

    // コントラスタ
    // クラスが new された瞬間に 最初に一度だけ実行される特別な関数
    public VehicleRobotState(VehicleRobotDynamics dyn) 
    {
        // ここでTargetPointStateDataクラスのインスタンス生成
        current = new VehicleRobotStateData();

        this.dynamics = dyn;
    }

    public void Initialize(VehicleKinematics v)
    {

        this.vehicle = v;

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
        current.u1 = vehicle.GetU1();
        current.u2 = vehicle.GetU2();
        current.u3 = vehicle.GetU3();

        runge.UpdateStateRungeKutta(dt);

        // 結果を State に反映する
        SetTime(runge.x_new[0]);
        SetX1(runge.x_new[1]); 
        SetY1(runge.x_new[2]); 
        SetPhi1(runge.x_new[3]);
        SetTheta1(runge.x_new[4]);
        SetPhi2(runge.x_new[5]);
        SetTheta2(runge.x_new[6]);
        SetTheta3(runge.x_new[7]);

        runge.CommitStep();
    }

    private float NormalizeAngle(float angle)
    {
        return math.atan2(math.sin(angle), math.cos(angle));
    }

    // Setter
    public void SetTime(float v) { current.t = v; }
    public void SetX1(float v) { current.x1 = v; }
    public void SetY1(float v) { current.y1 = v; }
    public void SetPhi1(float v) { current.phi1 = v; }
    public void SetTheta1(float v) { current.theta1 = v; }
    public void SetPhi2(float v) { current.phi2 = v; }
    public void SetTheta2(float v) { current.theta2 = v; }
    public void SetTheta3(float v) { current.theta3 = v; }

    // Getter
    public float GetTime() => current.t;
    public float GetX1() => current.x1;
    public float GetY1() => current.y1;
    public float GetPhi1() => current.phi1;
    public float GetTheta1() => current.theta1;
    public float GetPhi2() => current.phi2;
    public float GetTheta2() => current.theta2;
    public float GetTheta3() => current.theta3;

    public float GetU1() => current.u1;
    public float GetU2() => current.u2;
    public float GetU3() => current.u3;


    public Vector2 GetPosition()
    {
        Vector2 pos;

        pos.x = GetX1();
        pos.y = GetY1();

        return pos;
    }
}
