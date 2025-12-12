using Unity.Mathematics;
using UnityEngine;
using System.IO;
using System.Text;
using System.Globalization;

public static class OutputCSV
{
    public static void WriteBSplineCurve(
        float2[] past,
        // float2[] future,
        // float2[] merge,
        float2[] smooth,
        float2[] resample,
        // float2[] resample2,
        float2[] bspline,
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
                // "future.x,future.y," +
                // "merge.x,merge.y," +
                "smooth.x,smooth.y," +
                "resample.x,resample.y," +
                // "resample2.x,resample2.y," +
                "bspline.x,bspline.y," +
                "dist_smooth(dist from next)," +
                "dist_resample(dist from next)," +
                // "dist_resample2(dist from next)," +
                "dist_bspline(dist from next)"
            );

            int n = Mathf.Max(
                SafeLen(past),
                // SafeLen(future),
                // SafeLen(merge),
                SafeLen(resample),
                SafeLen(smooth),
                // SafeLen(resample2),
                SafeLen(bspline)
            );

            for (int i = 0; i < n; i++)
            {
                float s = i * ds;

                float2 pPast     = SafeGet(past, i);
                // float2 pFuture   = SafeGet(future, i);
                // float2 pMerge    = SafeGet(merge, i);
                float2 pResample = SafeGet(resample, i);
                float2 pSmooth   = SafeGet(smooth, i);
                // float2 pResample2 = SafeGet(resample2, i);
                float2 pBS       = SafeGet(bspline, i);

                // 距離計算（安全）
                float dResample = DistSafe(resample, i);
                float dSmooth   = DistSafe(smooth, i);
                // float dResample2 = DistSafe(resample2, i);
                float dBS       = DistSafe(bspline, i);

                writer.WriteLine(
                    $"{F(s)}," +
                    $"{F(pPast.x)},{F(pPast.y)}," +
                    // $"{F(pFuture.x)},{F(pFuture.y)}," +
                    // $"{F(pMerge.x)},{F(pMerge.y)}," +
                    $"{F(pSmooth.x)},{F(pSmooth.y)}," +
                    $"{F(pResample.x)},{F(pResample.y)}," +
                    // $"{F(pResample2.x)},{F(pResample2.y)}," +
                    $"{F(pBS.x)},{F(pBS.y)}," +
                    $"{F(dSmooth)}," +
                    $"{F(dResample)}," +
                    // $"{F(dResample2)}," +
                    $"{F(dBS)}"
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
