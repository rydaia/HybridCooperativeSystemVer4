using UnityEngine;

public struct TeleportPoint // ステージごとの移動場所
{
    public string stageName; // ステージ名
    public float x;
    public float y;
    public float theta; // 姿勢角（移動後は一直線）

    public TeleportPoint(string stageName,  float x, float y, float theta)
    {
        this.stageName = stageName;
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
}
public class TeleportConfig
{
    public static readonly TeleportPoint[] teleportPoints =
    {
        new TeleportPoint("practice", 0f, 0f, 0f),
        new TeleportPoint("stage1", -400f, 200f, 0f),
        new TeleportPoint("stage2", -400f, 200f, 0f),
    };
}
