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
            50f
        ),
        new TeleportCameraPoint("stage2",
            new Vector3(-290f, 50f, 100f),
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
        // Display2 を有効化
        if (Display.displays.Length > 1)
            Display.displays[1].Activate();
    }

    void Update()
    {
        HandleInput();
    }

    private void HandleInput()
    {
        for (int i = 0; i < TeleportCameraConfig.teleportPoints.Length; i++)
        {
            var alpha = (KeyCode)((int)KeyCode.Alpha1 + i);
            var keypad = (KeyCode)((int)KeyCode.Keypad1 + i);

            if (Input.GetKeyDown(alpha) || Input.GetKeyDown(keypad))
            {
                TeleportCamera(TeleportCameraConfig.teleportPoints[i]);
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
