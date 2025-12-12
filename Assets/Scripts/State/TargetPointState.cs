
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

public enum TargetPointMode
{
    Forward,   // v1 > 0
    Stop,      // |v1| < 閾値
    Back       // v1 < 0
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

    // DIM = 微分方程式の数（s, x, y, theta) の4つ
    private int DIM = 4; // 目標点用

    private float stoppedTheta = 0f;  // 停止時の姿勢角保持

    private TargetPointMode currentMode;     // 現在のモード
    private TargetPointMode prevMode;

    private float prevTotalS;

    // コントラスタ
    // クラスが new された瞬間に 最初に一度だけ実行される特別な関数
    public TargetPointState(TargetPointDynamics dyn) 
    {
        // ここでTargetPointStateDataクラスのインスタンス生成
        current = new TargetPointStateData();
        this.dynamics = dyn;
    }

    public void Initialize()
    {

        prevTotalS = 0.0f;

        // current.t = 0.0f;
        // current.x = -0.75f;
        // current.y = -0.7f;
        // current.t = 0.0f;

        setX(0.0f);
        setY(0.0f);
        setTheta(0.0f);
        setV1(0.0f);
        setV2(0.0f);

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

    public void updateTargetPointState(float dt)
    {
        // 入力は current.v1 / current.v2 にすでにセットされている

        // 積分前に現在のSを前回のSとして保存
        prevTotalS = getS();

        runge.UpdateStateRungeKutta(dt);

        // 結果を State に反映する
        current.t     = runge.x_new[0];
        current.s     = runge.x_new[1];
        current.x     = runge.x_new[2];
        current.y     = runge.x_new[3];




        // 目標点が止まった時を検知して、その時のthetaを一時保存
        // 停止状態時は、そのtheta前後30degしか回転できないようにする
        float v1 = current.v1;
        float theta_raw = NormalizeAngle(runge.x_new[4]);

        if (currentMode == TargetPointMode.Stop)
        {

            if (prevMode != TargetPointMode.Stop)
            {

                stoppedTheta = current.theta;
                Debug.Log($"目標点の状態：停止しました stoppedTheta:{stoppedTheta}");

            }

            float diff = NormalizeAngle(theta_raw - stoppedTheta);


            float limit = 30f * Mathf.Deg2Rad;
            diff = math.clamp(diff, -limit, limit);

            // 積分値自体に強制反映
            runge.x_new[4] = stoppedTheta + diff;
        }
        else
        {
            // 通常時は正規化のみ
            runge.x_new[4] = NormalizeAngle(theta_raw);
        }

        current.theta = runge.x_new[4];

        runge.CommitStep();
    }

    public void UpdateModeState(float v1)
    {
        const float eps = 1e-4f;

        if (v1 > eps) 
            SetCurrentMode(TargetPointMode.Forward);
        else if (v1 < -eps)
            SetCurrentMode(TargetPointMode.Back);

        else
            SetCurrentMode(TargetPointMode.Stop);

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


    private float NormalizeAngle(float angle)
    {
        return math.atan2(math.sin(angle), math.cos(angle));
    }

    // Setter
    public void setX(float _x) { current.x = _x; }
    public void setY(float _y) { current.y = _y; }
    public void setTheta(float _theta) { current.theta = _theta; }
    public void setV1(float _v1) { current.v1 = _v1; }
    public void setV2(float _v2) { current.v2 = _v2; }

    // Getter
    public float getTime() => current.t;
    public float getS() => current.s;
    public float getX() => current.x;
    public float getY() => current.y;
    public float getTheta() => current.theta;
    public float getV1() => current.v1;
    public float getV2() => current.v2;

    public float GetPrevTotalS() => prevTotalS;


    public Vector2 GetPosition()
    {
        Vector2 pos;

        pos.x = getX();
        pos.y = getY();

        return pos;
    }



    public float getKappa()
    {
        float v1, v2;

        // 停止状態時
        if(GetMode() == TargetPointMode.Stop)
        {
            // 仮にこの速度で走り始めた時
            v1 = 1.0f;
            v2 = getV2();  // ステア角速度

        }else
        {
            v1 = getV1();  // 前進速度
            v2 = getV2();  // ステア角速度
        }

        float kappa = v2 / v1;

        // 曲率の絶対値を上限で clamp（左右対称）
        const float kappa_max = 0.2f;
        kappa = math.clamp(kappa, -kappa_max, kappa_max);

        return kappa;
    }
}
