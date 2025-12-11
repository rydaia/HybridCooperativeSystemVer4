using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// 目標点の操作
// 上矢印：徐々に加速
// 下矢印：徐々に後退
// キーを離すと減速して 0 に近づく
public class VehicleRobotCtrl : MonoBehaviour
{
    [Header("w1の制御")]
    public float w1 = 0.0f;                  // 現在の速度
    public float maxW1 = 0.034124f;          // 最大前進スピード
    public float minW1 = -0.034124f;         // 最大後退スピード
    public float acceleration = 0.0034124f;  // 加速・減速量

    [Header("w3の制御")]
    public float w3 = 0.0f;                  // 現在の速度
    public float maxW3 = 0.2f;          // 最大前進スピード
    public float minW3 = -0.2f;         // 最大後退スピード
    public float steerAcceleration = 0.1f;  // 加速・減速量

    void Update()
    {
        HandleDriveInput();
        HandleSteerInput();
    }

    private void HandleDriveInput()
    {
        // 上矢印キー（前進）
        if (Input.GetKey(KeyCode.UpArrow))
        {
            // w1 += acceleration * Time.deltaTime;
            w1 += acceleration * 0.01f;
            w1 = Mathf.Clamp(w1, minW1, maxW1);
        }
        // 下矢印キー（後退）
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            w1 -= acceleration * 0.01f;
            // w1 -= acceleration * Time.deltaTime;
            w1 = Mathf.Clamp(w1, minW1, maxW1);
        }
        // どちらのキーも離したとき → 減速
        else
        {
            if (w1 > 0)
            {
                w1 -= acceleration * Time.deltaTime;
                if (w1 < 0) w1 = 0;
            }
            else if (w1 < 0)
            {
                w1 += acceleration * Time.deltaTime;
                if (w1 > 0) w1 = 0;
            }
        }
    }

    private void HandleSteerInput()
    {
        // 左矢印キー：左（ステア角速度をマイナス）
        // Lキー：右（ステア角速度をマイナス）
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            // w3 += steerAcceleration * Time.deltaTime;
            w3 += steerAcceleration * 0.01f;
            w3 = Mathf.Clamp(w3, minW3, maxW3);
        }
        // Lキー：右（ステア角速度をマイナス）
        // 左矢印キー：左（ステア角速度をプラス）
        else if (Input.GetKey(KeyCode.RightArrow))
        {
            // w3 -= steerAcceleration * Time.deltaTime;
            w3 -= steerAcceleration * 0.01f;

            w3 = Mathf.Clamp(w3, minW3, maxW3);
        }
        // 離したら徐々にステア角速度を0に戻す（自然な挙動）
        else
        {
            if (w3 > 0)
            {
                // w3 -= steerAcceleration * Time.deltaTime;
                w3 -= steerAcceleration * 0.01f;

                if (w3 < 0) w3 = 0;
            }
            else if (w3 < 0)
            {
                w3 += steerAcceleration * 0.01f;
                // w3 += steerAcceleration * Time.deltaTime;

                if (w3 > 0) w3 = 0;
            }
        }
    }

}
