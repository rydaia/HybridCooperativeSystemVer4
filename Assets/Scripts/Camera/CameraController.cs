// Scripts/Camera/CameraController.cs
// ステージごとに設定された俯瞰カメラ位置へ切り替え，表示範囲を調整するカメラ制御クラス

using UnityEngine;

[System.Serializable]
public struct TeleportCameraPoint
{
    public string stageName;
    public Vector3 position;
    public Vector3 eulerAngles; // 見下ろし角（回転）
    public float size;

    public TeleportCameraPoint(string stageName, Vector3 position, Vector3 eulerAngles, float size)
    {
        this.stageName = stageName;
        this.position = position;
        this.eulerAngles = eulerAngles;
        this.size = size;
    }
}

public static class TeleportCameraConfig
{
    public static readonly TeleportCameraPoint[] teleportPoints =
    {
        new TeleportCameraPoint("practice",
            new Vector3(0f, 100f, 0f),
            new Vector3(90f, 0f, 0f),
            50f
        ),
        new TeleportCameraPoint("stage1",
            new Vector3(-280f, 50f, 230f),
            new Vector3(90f, 0f, 0f),
            40f
        ),
        new TeleportCameraPoint("stage2",
            new Vector3(-275f, 50f, 97f),
            new Vector3(90f, 90f, 0f),
            26.0f
        ),

        new TeleportCameraPoint("stage3",
            new Vector3(-380f, 50f, 230f),
            new Vector3(90f, 0f, 0f),
            40f
        ),
        new TeleportCameraPoint("stage4",
            new Vector3(-390f, 50f, 100f),
            new Vector3(90f, 90f, 0f),
            50f
        ),

        new TeleportCameraPoint("stage5",
            new Vector3(-480f, 50f, 230f),
            new Vector3(90f, 0f, 0f),
            40f
        ),
        new TeleportCameraPoint("stage6",
            new Vector3(-490f, 50f, 100f),
            new Vector3(90f, 90f, 0f),
            50f
        ),

        new TeleportCameraPoint("stage7",
            new Vector3(-580f, 50f, 230f),
            new Vector3(90f, 0f, 0f),
            40f
        ),
        new TeleportCameraPoint("stage8",
            new Vector3(-590f, 50f, 100f),
            new Vector3(90f, 90f, 0f),
            50f
        ),
        new TeleportCameraPoint("stage9",
            new Vector3(-680f, 50f, 230f),
            new Vector3(90f, 0f, 0f),
            40f
        ),
        new TeleportCameraPoint("stage10",
            new Vector3(-690f, 50f, 100f),
            new Vector3(90f, 90f, 0f),
            50f
        ),
    };
}

public class CameraController : MonoBehaviour
{
    [Header("Top View")]
    [SerializeField] private Transform stageTop;
    [SerializeField] private Camera topCamera;

    void Awake()
    {
        if (topCamera != null)
        {
            topCamera.orthographic = true; // 俯瞰なら固定しちゃうと安全
        }
    }

    void Start()
    {
    }

    void Update()
    {
        HandleInput();
    }

    private void HandleInput()
    {

        if (Input.GetKeyDown(KeyCode.J))
        {
            TeleportCamera(TeleportCameraConfig.teleportPoints[0]);
        }

        for (int i = 0; i < TeleportCameraConfig.teleportPoints.Length; i++)
        {
            var alpha = (KeyCode)((int)KeyCode.Alpha1 + i);
            var keypad = (KeyCode)((int)KeyCode.Keypad1 + i);

            if (Input.GetKeyDown(alpha) || Input.GetKeyDown(keypad))
            {
                TeleportCamera(TeleportCameraConfig.teleportPoints[i+1]);
                break;
            }
        }
    }

    private void TeleportCamera(TeleportCameraPoint p)
    {
        if (stageTop == null || topCamera == null) return;

        stageTop.SetPositionAndRotation(
            p.position,
            Quaternion.Euler(p.eulerAngles)
        );

        topCamera.orthographicSize = p.size;
    }
}
