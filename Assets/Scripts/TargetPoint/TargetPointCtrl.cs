using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using UnityEngine.InputSystem;
using System;

// MonoBehavior
public class TargetPointCtrl : MonoBehaviour
{

    public enum InputMode 
    { 
        Keyboard, 
        G923 
    }

    [SerializeField] private InputMode inputMode;

    // G923用設定
    [Header("G923 (Input Actions)")]
    [SerializeField] private InputActionProperty g923Steer1; // ハンドル入力
    [SerializeField] private InputActionProperty g923Throttle; // アクセル入力
    [SerializeField] private InputActionProperty g923Brake; // ブレーキ
    [SerializeField] private InputActionProperty g923Mode; // モード切替
    [SerializeField] private InputActionProperty g923Reset; // 全リセット


    [Header("v1の制御")]
    public float maxV1;          // 最大前進スピード
    public float minV1;         // 最大後退スピード
    public float driveAcceleration;  // 加速・減速量

    [Header("v2の制御")]
    public float maxV2;          // 最大旋回スピード
    public float minV2;         // 最大旋回スピード
    public float steerAcceleration;  // 加速・減速量

    [Header("CalculationManager 参照")]
    public CalculationManager calc;

    private float _v1, _v2;
    [SerializeField] private float minCruiseSpeed; // 最低巡航速度
    [SerializeField] private float brakeDecel;     // ブレーキ減速率

    private bool InputV2Flag;
    private TargetPointMode nextDriveMode;

    void OnEnable()
    {
        g923Steer1.action?.Enable();
        g923Throttle.action?.Enable();
        g923Brake.action?.Enable();
        g923Mode.action?.Enable();
        g923Reset.action?.Enable();
    }

    void OnDisable()
    {
        g923Steer1.action?.Disable();
        g923Throttle.action?.Disable();
        g923Brake.action?.Disable();
        g923Mode.action?.Disable();
        g923Reset.action?.Disable();
    }

    void Update()
    {
        if (inputMode == InputMode.G923)
        {
            // Debug.Log(
            //     $"Steer={g923Steer1.action.ReadValue<float>()}, " +
            //     $"Throttle={g923Throttle.action.ReadValue<float>()}, " +
            //     $"Brake={g923Brake.action.ReadValue<float>()}"
            // );
        }
    }

    public void Initialize()
    {

        // inputMode = InputMode.Keyboard;
        inputMode = InputMode.G923;

        minCruiseSpeed = 1.0f; // 最低巡航速度
        brakeDecel = 2.0f;     // ブレーキ減速率

        maxV1 = 3.0f;          // 最大前進スピード
        minV1 = -3.0f;         // 最大後退スピード
        driveAcceleration = 0.5f;  // 加速・減速量

        maxV2 = 1.0f;          // 最大旋回スピード [rad/s]
        minV2 = -1.0f;         // 最大旋回スピード [rad/s]
        steerAcceleration = 0.25f;  // 加速・減速量

        InputV2Flag = false;
        nextDriveMode = TargetPointMode.Forward;
    }

    public void ReadInput(int i)
    {
        HandleModeChange();

        if (inputMode == InputMode.G923)
        {
            HandleDriveInput_G923();
            HandleSteerInput_G923();
        }
        else
        {
            HandleDriveInput();
            HandleSteerInput();
        }

        calc.targetPointState.UpdateTargetPoint(_v1);
    }

    // モードの切り替え
    private void HandleModeChange()
    {
        if (g923Mode.action.WasPressedThisFrame())
        {
            ChangeMode();
        }

        if (Input.GetKeyDown(KeyCode.P))
        {
            ChangeMode();
        }
    }

    private void ChangeMode()
    {
        if (!calc.targetPointState.GetIsStop()) 
        {
            Debug.Log("停止中でないためモード変更不可");
            return;
        }

        switch (nextDriveMode)
        {
            // Parkingの時は前進モードに遷移
            case TargetPointMode.Parking:
                calc.targetPointState.SetPrevMode(calc.targetPointState.GetMode());
                calc.targetPointState.SetCurrentMode(nextDriveMode);
                nextDriveMode = TargetPointMode.Forward;
                break;

            // 前進モードの時は後退モードに遷移
            case TargetPointMode.Forward:
                calc.targetPointState.SetPrevMode(calc.targetPointState.GetMode());
                calc.targetPointState.SetCurrentMode(nextDriveMode);
                // nextDriveMode = TargetPointMode.Back;
                nextDriveMode = TargetPointMode.Parking;
                break;


            // 後退モードの時はParkingモードに遷移
            case TargetPointMode.Back:
                calc.targetPointState.SetPrevMode(calc.targetPointState.GetMode());
                calc.targetPointState.SetCurrentMode(nextDriveMode);
                nextDriveMode = TargetPointMode.Parking;
                break;
        }

        Debug.Log("Mode changed to: " + calc.targetPointState.GetMode());
    }

    private void HandleDriveInput_G923()
    {
        float throttle = Mathf.Clamp01(g923Throttle.action.ReadValue<float>());
        float brake    = Mathf.Clamp01(g923Brake.action.ReadValue<float>());

        Debug.Log($"brake: {brake}, throttle:{throttle}");

        // float dt = Time.deltaTime;
        float dt = 0.01f;

        // 現在速度を取得
        _v1 = calc.targetPointState.getV1();

        var mode = calc.targetPointState.GetMode();
        // 前進モードはスピードが0の時も存在する.
        if(mode == TargetPointMode.Forward)
        {
            // アクセル
            if (throttle > 0.01f)
            {
                // アクセル量に応じて加速
                _v1 += throttle * driveAcceleration * dt;
            }
            else
            {
                // アクセルを離したら自然減速（最低速度へ）
                _v1 -= driveAcceleration * dt;
            }

            // ブレーキ
            if (brake > 0.01f)
            {
                // ブレーキは強制的に減速（0 まで許可）
                _v1 -= brake * brakeDecel * dt;
            }

            // 制約
            if (brake <= 0.01f)
            {
                // ブレーキを踏んでいない限り 0.1 以下にはならない
                _v1 = Mathf.Max(_v1, minCruiseSpeed);
            }
            else
            {
                // ブレーキ中は 0 までOK
                _v1 = Mathf.Max(_v1, 0f);
            }
        }
        else if(mode == TargetPointMode.Back)
        {
            // // 後退処理
            // if (Input.GetKey(KeyCode.UpArrow))
            // {
            //     // アクセル：加速
            //     _v1 += driveAcceleration * 0.01f;
            // }
            // else if (Input.GetKey(KeyCode.DownArrow))
            // {
            //     // ブレーキ：減速（0未満にはしない）
            //     _v1 -= driveAcceleration * 0.01f;
            //     if (_v1 < 0) _v1 = 0;
            // }
            // else
            // {
            //     // 何も押してないときは自然減速
            //     // 0.1以下の場合は0.1になるように加速
            //     _v1 -= driveAcceleration * Time.deltaTime;

            //     if (_v1 < 0.1f)
            //     {
            //         _v1 = 0.1f;
            //     }
            // }
        }
        else
        {
            if (Input.GetKey(KeyCode.UpArrow))
            {
                Debug.Log("Parkingモードのため入力できません.");
            }
        }

        _v1 = Mathf.Clamp(_v1, 0f, maxV1);

        calc.targetPointState.setV1(_v1);
    }


    // スピードが0の時を停止状態とする.
    private void HandleDriveInput()
    {

        _v1 = calc.targetPointState.getV1();

        var mode = calc.targetPointState.GetMode();
        // 前進モードはスピードが0の時も存在する.
        if(mode == TargetPointMode.Forward)
        {
            // 前進処理
            // 上キーを押し続けたら加速.
            // 離したら0に収束
            // 下キーはブレーキ
            // 上キーを離しても減速するし、下キーを押しても減速
            // 下限は0
            // 一旦後退は無視して良い
            if (Input.GetKey(KeyCode.UpArrow))
            {
                // アクセル：加速
                _v1 += driveAcceleration * 0.01f;
            }
            else if (Input.GetKey(KeyCode.DownArrow))
            {
                // ブレーキ：減速（0未満にはしない）
                _v1 -= driveAcceleration * 0.01f;
                if (_v1 < 0) _v1 = 0;
            }
            else
            {
                // 何も押してないときは自然減速
                // 0.1以下の場合は0.1になるように加速
                _v1 -= driveAcceleration * Time.deltaTime;

                if (_v1 < 0.1f)
                {
                    _v1 = 0.1f;
                }
            }
        }
        else if(mode == TargetPointMode.Back)
        {
            // 後退処理
            if (Input.GetKey(KeyCode.UpArrow))
            {
                // アクセル：加速
                _v1 += driveAcceleration * 0.01f;
            }
            else if (Input.GetKey(KeyCode.DownArrow))
            {
                // ブレーキ：減速（0未満にはしない）
                _v1 -= driveAcceleration * 0.01f;
                if (_v1 < 0) _v1 = 0;
            }
            else
            {
                // 何も押してないときは自然減速
                // 0.1以下の場合は0.1になるように加速
                _v1 -= driveAcceleration * Time.deltaTime;

                if (_v1 < 0.1f)
                {
                    _v1 = 0.1f;
                }
            }
        }
        else
        {
            if (Input.GetKey(KeyCode.UpArrow))
            {
                Debug.Log("Parkingモードのため入力できません.");
            }
        }

        calc.targetPointState.setV1(Mathf.Clamp(_v1, minV1, maxV1));
    }

    private void HandleSteerInput_G923()
    {
        float steer = g923Steer1.action.ReadValue<float>(); // -1 ～ +1

        _v2 = steer * maxV2;

        InputV2Flag = Mathf.Abs(steer) > 0.01f;

        calc.targetPointState.setV2(Mathf.Clamp(_v2, minV2, maxV2));
    }


    private void HandleSteerInput()
    {
        _v2 = calc.targetPointState.getV2();

        if (Input.GetKey(KeyCode.LeftArrow))
        {
            // _v2 += steerAcceleration * Time.deltaTime;
            InputV2Flag = true;
            _v2 += steerAcceleration * 0.01f;
        }
        else if (Input.GetKey(KeyCode.RightArrow))
        {
            // _v2 -= steerAcceleration * Time.deltaTime;
            InputV2Flag = true;
            _v2 -= steerAcceleration * 0.01f;
        }
        else
        {
            InputV2Flag = false;

            if (_v2 > 0)
            {
                // _v2 -= steerAcceleration * Time.deltaTime;
                _v2 -= steerAcceleration * 0.01f;

                if (_v2 < 0) _v2 = 0;
            }
            else if (_v2 < 0)
            {
                _v2 += steerAcceleration * 0.01f;
                // _v2 += steerAcceleration * Time.deltaTime;

                if (_v2 > 0) _v2 = 0;
            }
        }

        calc.targetPointState.setV2(Mathf.Clamp(_v2, minV2, maxV2));
    }

    public float GetMaxSpeed() => maxV1;
    public bool GetInPutV2Flag() => InputV2Flag;


}

