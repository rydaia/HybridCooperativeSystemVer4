using Unity.Mathematics;
using UnityEngine;
using System.IO;
using System.Text;
using System.Globalization;

public static class OutputCSV
{
    public static void WriteBSplineCurve(
        float2[] past,
        float2[] smooth,
        float2[] resample,
        float[] arrayU,
        float2[] frontBspline,
        float2[] rearBspline,
        float2[] frontDerivative1,
        float2[] rearDerivative1,
        float ds,
        float time)
    {
        string safeTime = time.ToString().Replace('.', '_');
        string fileName = $"Bspline_output_{safeTime}.csv";

        using (var writer = new StreamWriter(fileName, false, Encoding.GetEncoding("Shift_JIS")))
        {
            // ---- ヘッダー ----
            writer.WriteLine(
                "s," +
                "past.x,past.y," +
                "smooth.x,smooth.y," +
                "resample.x,resample.y," +
                "index," +
                "u," +
                "frontBspline.x,frontBspline.y," +
                "rearBspline.x,rearBspline.y," +
                "frontDerivative1.x,frontDerivative1.y," +
                "rearDerivative1.x,rearDerivative1.y," +
                "dist_smooth(dist from next)," +
                "dist_resample(dist from next)," +
                "dist_bspline(dist from next)," +
                "cs1"
            );

            int n = Mathf.Max(
                SafeLen(past),
                SafeLen(resample),
                SafeLen(smooth),
                SafeLen(frontBspline),
                SafeLen(rearBspline),
                SafeLen(frontDerivative1),
                SafeLen(rearDerivative1)
            );

            for (int i = 0; i < n; i++)
            {
                float s = i * ds;

                float2 pPast     = SafeGet(past, i);
                float2 pResample = SafeGet(resample, i);
                float2 pSmooth   = SafeGet(smooth, i);
                float2 pFB       = SafeGet(frontBspline, i);
                float2 pRB       = SafeGet(rearBspline, i);
                float2 pFD       = SafeGet(frontDerivative1, i);
                float2 pRD       = SafeGet(rearDerivative1, i);

                // 距離計算（安全）
                float dResample = DistSafe(resample, i);
                float dSmooth   = DistSafe(smooth, i);
                float dFB       = DistSafe(frontBspline, i);

                float u = arrayU[i];

        //         float _squaredD1rx1du11 = Mathf.Pow(pFD.x,2);
        //         float _squaredD1ry1du11 = Mathf.Pow(pFD.y,2);

        //         float _formulaOfD1ry1du11MulD2rx1du12 = pFD.y*pDerivative2.x;
        //         float _formulaOfD1rx1du11MulD2ry1du12 = pDerivative1.x*pDerivative2.y;

        //         float _formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11 = _squaredD1rx1du11 + _squaredD1ry1du11;

        //         float frontCs1 =  -((_squaredD1ry1du11*(_formulaOfD1ry1du11MulD2rx1du12 - _formulaOfD1rx1du11MulD2ry1du12)) / Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 2.5f))
        // + (_squaredD1rx1du11*(-(_formulaOfD1ry1du11MulD2rx1du12) + _formulaOfD1rx1du11MulD2ry1du12)) / Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 2.5f);

                writer.WriteLine(
                    $"{F(s)}," +
                    $"{F(pPast.x)},{F(pPast.y)}," +
                    $"{F(pSmooth.x)},{F(pSmooth.y)}," +
                    $"{F(pResample.x)},{F(pResample.y)}," +
                    $"{F(i)}," +
                    $"{F(u)}," +
                    $"{F(pFB.x)},{F(pFB.y)}," +
                    $"{F(pRB.x)},{F(pRB.y)}," +
                    $"{F(pFD.x)},{F(pFD.y)}," +
                    $"{F(pRD.x)},{F(pRD.y)}," +
                    $"{F(dSmooth)}," +
                    $"{F(dResample)}," +
                    $"{F(dFB)}"
                );
            }
        }

        Debug.Log($"CSV 出力完了: {fileName}");
    }


    // ---- 距離（i→i+1）安全取得 ----
    static float DistSafe(float2[] arr, int i)
    {
        if (arr == null || arr.Length < 2) return float.NaN;
        if (i >= arr.Length - 1) return float.NaN;
        return math.distance(arr[i], arr[i + 1]);
    }


    // ---- NaN 安全な float2 取得 ----
    static float2 SafeGet(float2[] arr, int i)
    {
        if (arr == null || i >= arr.Length)
            return new float2(float.NaN, float.NaN);
        return arr[i];
    }

    // ---- 配列長が null でも安全 ----
    static int SafeLen(float2[] arr)
    {
        return (arr == null ? 0 : arr.Length);
    }

    // ---- NaN の時は "" を返す ----
    static string F(float value)
    {
        return float.IsNaN(value) ? "" : value.ToString("F6", CultureInfo.InvariantCulture);
    }
}
