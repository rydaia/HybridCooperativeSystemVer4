// Scripts/ReadCSV.cs
// Resourcesフォルダ内のCSVファイルから速度データ（float2配列）を読み込み，制御やシミュレーションに利用するクラス

using UnityEngine;
using Unity.Mathematics;
using System.IO;
using System.Globalization;

// 制御点データの読み込み
// ファイル名：control_points.csv
// 列はs(0.01刻み),x, y 
// 制御点の個数は1202個(過去データ1001個+未来データ200個 + 1)
public static class ReadCSV
{
    // Assets/Resources/control_points.csv に置くと Resources.Load が使える
    const string FileName = "input_speed";

    public static float2[] ReadSpeedData()
    {
        // Resources フォルダから読み込む
        TextAsset csvFile = Resources.Load<TextAsset>(FileName);

        if (csvFile == null)
        {
            Debug.LogError("CSVファイルが取得できません: " + FileName);
            return null;
        }

        string[] lines = csvFile.text.Split('\n');

        int start = 1;  // 1行目はヘッダー
        int count = lines.Length - start;

        float2[] arr = new float2[count];

        for (int i = start; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] cols = lines[i].Split(',');

            float x = float.Parse(cols[1], CultureInfo.InvariantCulture);
            float y = float.Parse(cols[2], CultureInfo.InvariantCulture);

            arr[i - start] = new float2(x, y);
        }


        Debug.Log($"速度ファイルの読み込みが終了: {arr.Length} 点");

        return arr;
    }
}
