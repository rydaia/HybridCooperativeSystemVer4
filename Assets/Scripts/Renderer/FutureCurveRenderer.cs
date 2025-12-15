using UnityEngine;
using Unity.Mathematics;

[RequireComponent(typeof(LineRenderer))]
public class FutureCurveRenderer : MonoBehaviour
{
    public CalculationManager calc;

    LineRenderer lr;
    Vector3[] renderBuffer;
    int sampleCount = 1000;

    void Awake()
    {
        lr = GetComponent<LineRenderer>();
        lr.positionCount = sampleCount;

        lr.startWidth = 0.2f;
        lr.endWidth   = 0.2f;

        renderBuffer = new Vector3[sampleCount];
    }

    void LateUpdate()
    {
        if (calc == null || calc.cpQueue == null) return;

        var native = calc.cpQueue.GetFutureNative();

        int total = native.Length;

        if (total == 0) return;

        // 1000点に間引いて描画
        for (int i = 0; i < sampleCount; i++)
        {
            float t = (float)i / (sampleCount - 1);
            int idx = (int)(t * (total - 1));  // [0, total-1]
            float2 p = native[idx];

            renderBuffer[i] = new Vector3(p.x, 0.05f, p.y);
        }

        lr.SetPositions(renderBuffer);
    }
}
