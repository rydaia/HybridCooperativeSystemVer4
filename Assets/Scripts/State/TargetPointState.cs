
using System;
using UnityEngine;
using Unity.Mathematics;


public class TargetPointStateData : State
{
    public float s;
    public float x;
    public float y;
    public float theta;
    public float v1;     // 前進速度
    public float v2;     // ステア角速度
}

// Parking   ：v1=0 固定（ギアがP）
// Forward   ：v1>=0（前進）
// Back      ：v1<=0（後退）
public enum TargetPointMode
{
    Forward,   // v1 > 0
    Back,       // v1 < 0
    Parking
}

public enum CurrentTargetPoint
{
    Tp1,
    Tp2
}

// 目標点の状態更新はTargetPointStateクラスからRungeクラスを通して更新
// このクラスはCalculationMagerから呼び出される
public class TargetPointState
{

    // フィールド
    // TargetPointStateDataクラスのインスタンス
    public TargetPointStateData current;
    private TargetPointDynamics dynamics;
    private RungeKutta runge;
    private TargetPointCtrl TpCtrl;
    private VehicleRobotState vehicle;

    // DIM = 微分方程式の数（s, x, y, theta) の4つ
    private int DIM = 4; // 目標点用

    private float stoppedTheta = 0f;  // 停止時の姿勢角保持

    private TargetPointMode currentMode;     // 現在のモード
    private TargetPointMode prevMode;

    private CurrentTargetPoint currentTargetPoint;
    private CurrentTargetPoint prevTargetPoint;


    private float prevTotalS;

    private bool isStop;   // v1=0 でのみ true
    private bool prevIsStop;

    // コントラスタ
    // クラスが new された瞬間に 最初に一度だけ実行される特別な関数
    public TargetPointState(TargetPointDynamics dyn) 
    {
        // ここでTargetPointStateDataクラスのインスタンス生成
        current = new TargetPointStateData();
        this.dynamics = dyn;
    }

    public void Initialize(TargetPointCtrl tpctrl, VehicleRobotState vehicle)
    {

        SetIsStop(true);
        SetPrevIsStop(true);

        SetCurrentMode(TargetPointMode.Parking);
        SetPrevMode(TargetPointMode.Forward);

        this.TpCtrl = tpctrl;
        this.vehicle = vehicle;

        prevTotalS = 0.0f;

        // 初期化時は前方の目標点を利用
        SetCurrentTargetPoint(CurrentTargetPoint.Tp1);

        SetTime(0.0f);
        SetS(0.0f);


        // Tp1だったら先頭車両前輪間中点に設定
        // Tp2だったら後方車両後輪間中点に設定
        if(GetCurrentTargetPoint() == CurrentTargetPoint.Tp1)
        {
            float x = vehicle.GetMidpointBetweenFrontWheelsOfFV().x;
            float y = vehicle.GetMidpointBetweenFrontWheelsOfFV().y;

            // SetX(x);
            // SetY(y);

            SetX(-3.0f);
            SetY(0.0f);
        }
        else
        {
            float x = vehicle.GetMidpointBetweenRearWheelsOfSV().x;
            float y = vehicle.GetMidpointBetweenRearWheelsOfSV().y;

            SetX(x);
            SetY(y);
        }

        SetTheta(0.0f);
        SetV1(0.0f);
        SetV2(0.0f);

        runge = new RungeKutta(DIM, new Func<float[], float>[] {
            (x) => dynamics.f0(current),
            (x) => dynamics.f1(current),
            (x) => dynamics.f2(current),
            (x) => dynamics.f3(current),
            (x) => dynamics.f4(current),
        });

        runge.x_old[0] = current.t;
        runge.x_old[1] = current.s;
        runge.x_old[2] = current.x;
        runge.x_old[3] = current.y;
        runge.x_old[4] = current.theta;
    }

    public void Reset(VehicleRobotState vehicle)
    {

        SetIsStop(true);
        SetPrevIsStop(true);

        SetCurrentMode(TargetPointMode.Parking);
        SetPrevMode(TargetPointMode.Forward);

        prevTotalS = 0.0f;

        // 初期化時は前方の目標点を利用
        SetCurrentTargetPoint(CurrentTargetPoint.Tp1);

        SetTime(0.0f);
        SetS(0.0f);

        // Tp1だったら先頭車両前輪間中点に設定
        // Tp2だったら後方車両後輪間中点に設定
        if(GetCurrentTargetPoint() == CurrentTargetPoint.Tp1)
        {
            float x = vehicle.GetMidpointBetweenFrontWheelsOfFV().x;
            float y = vehicle.GetMidpointBetweenFrontWheelsOfFV().y;

            // SetX(x);
            // SetY(y);

            SetX(0.0f);
            SetY(0.0f);
        }
        else
        {
            float x = vehicle.GetMidpointBetweenRearWheelsOfSV().x;
            float y = vehicle.GetMidpointBetweenRearWheelsOfSV().y;

            SetX(x);
            SetY(y);
        }

        SetTheta(0.0f);
        SetV1(0.0f);
        SetV2(0.0f);

        runge.x_old[0] = GetTime();
        runge.x_old[1] = GetS();
        runge.x_old[2] = GetX();
        runge.x_old[3] = GetY();
        runge.x_old[4] = GetTheta();
    }


    public void updateTargetPointState(float dt)
    {
        // 入力は current.v1 / current.v2 にすでにセットされている

        // 積分前に現在のSを前回のSとして保存
        prevTotalS = GetS();

        // 現在の状態を元に積分
        // 目標点切り替えに対応
        runge.x_old[2] = GetX();
        runge.x_old[3] = GetY();
        runge.x_old[4] = GetTheta();

        runge.UpdateStateRungeKutta(dt);

        // 結果を State に反映する
        SetTime(runge.x_new[0]);
        SetS(runge.x_new[1]);
        SetX(runge.x_new[2]);
        SetY(runge.x_new[3]);

        // 目標点が止まった時を検知して、その時のthetaを一時保存
        // 停止状態時は、そのtheta前後30degしか回転できないようにする
        float v1 = current.v1;
        float theta_raw = NormalizeAngle(runge.x_new[4]);

        if (GetIsStop())
        {
            if (GetPrevIsStop() != GetIsStop())
            {
                stoppedTheta = current.theta;
                Debug.Log($"停止状態になったので保存: stoppedTheta={stoppedTheta}");
            }

            float diff = NormalizeAngle(theta_raw - stoppedTheta);
            float limit = 30f * Mathf.Deg2Rad;
            diff = math.clamp(diff, -limit, limit);

            runge.x_new[4] = stoppedTheta + diff;
        }

        SetTheta(runge.x_new[4]);

        runge.CommitStep();
    }

    public void UpdateTargetPoint(float v1)
    {

        UpdateStopFlag(v1);

        TargetPointMode current = GetMode();
        TargetPointMode prev = GetPrevMode();

        CurrentTargetPoint newTp = CurrentTargetPoint.Tp1;

        if (current == TargetPointMode.Forward)
        {
            newTp = CurrentTargetPoint.Tp1;
        }
        else if (current == TargetPointMode.Back)
        {
            newTp = CurrentTargetPoint.Tp2;
        }
        // Parkingの時
        else
        {
            // 前の状態で判断
            if(prev == TargetPointMode.Forward)
            {
                newTp = CurrentTargetPoint.Tp1;
            }
            else if(prev == TargetPointMode.Back)
            {
                newTp = CurrentTargetPoint.Tp2;
            }
            else
            {
                Debug.Log($"prev:{prev}, current:{current} どちらともParkingになっています.");
            }
        }

        // 2. 切り替わった場合のみ ChangeCurrentTargetPoint を呼ぶ
        if (newTp != GetCurrentTargetPoint())
        {
            SetPrevTargetPoint(GetCurrentTargetPoint());
            SetCurrentTargetPoint(newTp);

            ChangeCurrentTargetPoint(newTp);
        }
    }

    private void UpdateStopFlag(float v1)
    {
        // v1が0なら停止フラグON
        if (Mathf.Abs(v1) < 1e-3f)
        {
            // 現在のフラグ状態を前の状態として保存
            SetPrevIsStop(GetIsStop());
            SetIsStop(true);
        }
        else
        {
            // 0より大きくなった瞬間にStop解除
            if (GetIsStop())
            {
                // 現在のフラグ状態を前の状態として保存
                SetPrevIsStop(GetIsStop());
                SetIsStop(false);
                // v1 = 0.1f;  // 最低速度確保
            }
        }
    }

    public void ChangeCurrentTargetPoint(CurrentTargetPoint Tp)
    {

        // Debug.Log($"ChangeCurrentTargetPoint(CurrentTargetPoint Tp))");

        // ForwardでTp1
        // 現在のモードがFarwardつまりBackからの切り替え時
        if(Tp == CurrentTargetPoint.Tp1)
        {
            // 現在の目標点の座標をTp1に転換
            ChanegeTagetPointToTp1();
        }

        // BackでTp2
        // 現在のモードがつまりBackからの切り替え時
        if(Tp == CurrentTargetPoint.Tp2)
        {
            // 現在の目標点の座標をTp2に転換
            ChanegeTagetPointToTp2();
        }
    }

    // Tp2 → Tp1
    public void ChanegeTagetPointToTp1()
    {

        Debug.Log($"ChanegeTagetPointToTp1()");

        SetX(vehicle.GetMidpointBetweenFrontWheelsOfFV().x);
        SetY(vehicle.GetMidpointBetweenFrontWheelsOfFV().y);
        SetTheta(vehicle.GetTheta1());
    }


    // Tp1 → Tp2
    public void ChanegeTagetPointToTp2()
    {
        Debug.Log($"ChanegeTagetPointToTp2()");

        // Debug.Log($"vehicle.GetMidpointBetweenRearWheelsOfFV().x:{vehicle.GetMidpointBetweenRearWheelsOfFV().x}");
        // Debug.Log($"vehicle.GetMidpointBetweenRearWheelsOfFV().y:{vehicle.GetMidpointBetweenRearWheelsOfFV().y}");

        // Debug.Log($"vehicle.GetMidpointBetweenRearWheelsOfSV().x:{vehicle.GetMidpointBetweenRearWheelsOfSV().x}");
        // Debug.Log($"vehicle.GetMidpointBetweenRearWheelsOfSV().y:{vehicle.GetMidpointBetweenRearWheelsOfSV().y}");


        SetX(vehicle.GetMidpointBetweenRearWheelsOfSV().x);
        SetY(vehicle.GetMidpointBetweenRearWheelsOfSV().y);
        SetTheta(vehicle.GetTheta3());

        // Debug.Log($"getX():{getX()}, getY():{getY()}");
    }

    public void SetIsStop(bool v)
    {
        isStop = v;
    }

    public bool GetIsStop()
    {
        return isStop;
    }


    public void SetPrevIsStop(bool v)
    {
        prevIsStop = v;
    }

    public bool GetPrevIsStop()
    {
        return prevIsStop;
    }



    public void SetCurrentMode(TargetPointMode mode)
    {
        currentMode = mode;
    }

    public TargetPointMode GetMode()
    {
        return currentMode;
    }

    public TargetPointMode GetPrevMode()
    {
        return prevMode;
    }


    public void SetPrevMode(TargetPointMode mode)
    {
        prevMode = mode;
    }

    public void SetCurrentTargetPoint(CurrentTargetPoint v)
    {
        currentTargetPoint = v;
    }

    public CurrentTargetPoint GetCurrentTargetPoint()
    {
        return currentTargetPoint;
    }

    public void SetPrevTargetPoint(CurrentTargetPoint v)
    {
        prevTargetPoint = v;
    }

    public CurrentTargetPoint GetPrevTargetPoint()
    {
        return prevTargetPoint;
    }

    private float NormalizeAngle(float angle)
    {
        return math.atan2(math.sin(angle), math.cos(angle));
    }

    // Setter
    public void SetTime(float v) { current.t = v; }
    public void SetS(float v) { current.s = v; }
    public void SetX(float _x) { current.x = _x; }
    public void SetY(float _y) { current.y = _y; }
    public void SetTheta(float _theta) { current.theta = _theta; }
    public void SetV1(float _v1) { current.v1 = _v1; }
    public void SetV2(float _v2) { current.v2 = _v2; }

    // Getter
    public float GetTime() => current.t;
    public float GetS() => current.s;
    public float GetX() => current.x;
    public float GetY() => current.y;
    public float GetTheta() => current.theta;
    public float GetV1() => current.v1;
    public float GetV2() => current.v2;

    public float GetPrevTotalS() => prevTotalS;


    public Vector2 GetPosition()
    {
        Vector2 pos;

        pos.x = GetX();
        pos.y = GetY();

        return pos;
    }


    public float GetKappa()
    {
        float v1, v2;

        // 停止状態時
        if(GetIsStop())
        {
            // 仮にこの速度で走り始めた時
            v1 = 1.0f;
            v2 = GetV2();  // ステア角速度

        }else
        {
            v1 = GetV1();  // 前進速度
            v2 = GetV2();  // ステア角速度
        }

        float kappa = v2 / v1;

        // 曲率の絶対値を上限で clamp（左右対称）
        const float kappa_max = 0.2f;
        kappa = math.clamp(kappa, -kappa_max, kappa_max);

        return kappa;
    }
}
