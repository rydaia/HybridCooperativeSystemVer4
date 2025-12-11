using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

// MonoBehavior
public class TargetPointCtrl : MonoBehaviour
{
    [Header("v1の制御")]
    public float maxV1 = 1.0f;          // 最大前進スピード
    public float minV1 = -1.0f;         // 最大後退スピード
    public float driveAcceleration = 0.75f;  // 加速・減速量

    [Header("v2の制御")]
    public float maxV2 = 0.2f;          // 最大旋回スピード
    public float minV2 = -0.2f;         // 最大旋回スピード
    public float steerAcceleration = 0.1f;  // 加速・減速量

    [Header("CalculationManager 参照")]
    public CalculationManager calc;

    private float _v1, _v2;

    private float2[] arr;


    public void Initialize()
    {
        var speedData = ReadCSV.ReadSpeedData(); // ここは float2[] を返している前提

        arr = new float2[speedData.Length];

        for (int i = 0; i < speedData.Length; i++)
            arr[i] = speedData[i];
    }

    public void ReadInput(int i)
    {
        HandleDriveInput();
        HandleSteerInput();

        // float2 v = arr[i];
        // calc.targetPointState.setV1(v.x);
        // calc.targetPointState.setV2(v.y);

        calc.targetPointState.UpdateModeState(_v1);
    }




    private void HandleDriveInput()
    {
        _v1 = calc.targetPointState.getV1();

        // 上矢印キー（前進）
        if (Input.GetKey(KeyCode.UpArrow))
        {
            // _v1 += driveAcceleration * Time.deltaTime;
            _v1 += driveAcceleration * 0.01f;
        }
        // 下矢印キー（後退）
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            _v1 -= driveAcceleration * 0.01f;
            // _v1 -= driveAcceleration * Time.deltaTime;
        }
        // どちらのキーも離したとき → 減速
        else
        {
            if (_v1 > 0)
            {
                _v1 -= driveAcceleration * Time.deltaTime;
                if (_v1 < 0) _v1 = 0;
            }
            else if (_v1 < 0)
            {
                _v1 += driveAcceleration * Time.deltaTime;
                if (_v1 > 0) _v1 = 0;
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
            _v2 += steerAcceleration * 0.01f;
        }
        else if (Input.GetKey(KeyCode.RightArrow))
        {
            // _v2 -= steerAcceleration * Time.deltaTime;
            _v2 -= steerAcceleration * 0.01f;
        }
        else
        {
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

}

