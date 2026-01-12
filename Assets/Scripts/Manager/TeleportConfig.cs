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

    private TeleportPoint currentTeleport;

    public static readonly TeleportPoint[] teleportPoints =
    {
        new TeleportPoint("practice", 0f, 0f, 0f),
        new TeleportPoint("stage1-10", -310f, 215f, 0f),
        new TeleportPoint("stage2-10", -328f, 114f, 0f),
        new TeleportPoint("stage3-7.5", -410f, 215f, 0f),
        new TeleportPoint("stage4-7.5", -428f, 114f, 0f),
        new TeleportPoint("stage5-6", -510f, 215f, 0f),
        new TeleportPoint("stage6-6", -528f, 114f, 0f),
        new TeleportPoint("stage7-7", -610f, 215f, 0f),
        new TeleportPoint("stage8-7", -628f, 114f, 0f),
        new TeleportPoint("stage9-6.5", -710f, 215f, 0f),
        new TeleportPoint("stage10-6.5", -728f, 114f, 0f),
    };

    public void SetCurrentStage(int index)
    {
        currentTeleport = teleportPoints[index];
    }

    public TeleportPoint GetCurrentStage()
    {
        return currentTeleport;
    }
}
