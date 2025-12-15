using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

// MonoBehavior
public class TargetPointCtrl : MonoBehaviour
{
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

    private float2[] arr;

    private bool InputV2Flag;
    private TargetPointMode nextDriveMode;

    public void Initialize()
    {

        maxV1 = 1.2f;          // 最大前進スピード
        minV1 = -1.2f;         // 最大後退スピード
        driveAcceleration = 0.15f;  // 加速・減速量

        maxV2 = 0.2f;          // 最大旋回スピード
        minV2 = -0.2f;         // 最大旋回スピード
        steerAcceleration = 0.1f;  // 加速・減速量

        InputV2Flag = false;

        nextDriveMode = TargetPointMode.Forward;

    }

    public void ReadInput(int i)
    {

        HandleModeChange();

        HandleDriveInput();

        calc.targetPointState.UpdateTargetPoint(_v1);

        HandleSteerInput();
    }

    // モードの切り替え
    private void HandleModeChange()
    {
        if (Input.GetKeyDown(KeyCode.P))
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

