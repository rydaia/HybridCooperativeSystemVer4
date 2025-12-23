// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// using Unity.Cinemachine;

// public class CameraController : MonoBehaviour {

//     [Header("Camera References")]
//     [SerializeField] private CinemachineCamera followCamera;

    
//     [Header("Target References")]
//     [SerializeField] private Transform followTarget;
    
    
//     [Header("Camera Settings")]
//     [SerializeField] private float followDistance;
//     [SerializeField] private float followHeight;
    

//     void Start() {
//         followDistance = 3f;
//         followHeight = 4f;
        
//         SetupCameras();
        
//     }

//     // Update is called once per frame
//     void Update() {
//     }

//     // 各々のカメラモードの処理内容
//     void SetupCameras() {
//         if (followTarget == null) return;

//         SetupFollowViewCamera();
//     }


//     void SetupFollowViewCamera()
//     {
//         if (followCamera == null || followTarget == null) return;

//         followCamera.Follow = followTarget;
//         followCamera.LookAt = followTarget;

//         var follow = followCamera.GetComponent<CinemachineFollow>();
//         if (follow != null)
//         {
//             follow.FollowOffset = new Vector3(0, 15, -10);
//         }
//     }
// }
