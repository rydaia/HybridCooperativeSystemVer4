using System.Collections.Generic;
using UnityEngine;

public sealed class HitMarkerRoot : MonoBehaviour
{


    [SerializeField] GameObject dotPrefab;     // 1個でOK（赤点、Collider無し推奨）
    [SerializeField] LayerMask obstacleMask;

    [SerializeField] float dotScale = 0.05f;
    [SerializeField] float offset = 0.003f;

    readonly HashSet<ulong> touching = new();

    void Awake()
    {
        // 車両配下の全部位Colliderに中継を付与（IsTriggerは手動設定を前提）
        foreach (var col in GetComponentsInChildren<Collider>(true))
            if (!col.TryGetComponent(out HitMarkerPart _))
                col.gameObject.AddComponent<HitMarkerPart>();
    }

        // 「障害物に“初めて”当たった」時だけ true
    internal bool TryEnter(Collider self, Collider other)
    {
        if (((1 << other.gameObject.layer) & obstacleMask.value) == 0) return false;

        ulong key = PairKey(self.GetInstanceID(), other.GetInstanceID());
        if (!touching.Add(key)) return false; // 離れるまで1回

        // 点生成はPrefabがある時だけ（判定自体は動く）
        if (dotPrefab)
        {
            Vector3 pCar = self.ClosestPoint(other.bounds.center);
            Vector3 pObs = other.ClosestPoint(self.bounds.center);

            Spawn(dotPrefab, PushOut(pCar, pCar - pObs), self.transform); // 車に追従
            Spawn(dotPrefab, PushOut(pObs, pObs - pCar), null);          // ワールド固定
        }

        return true;
    }



    // internal void Enter(Collider self, Collider other)
    // {
    //     if (!dotPrefab) return;
    //     if (((1 << other.gameObject.layer) & obstacleMask.value) == 0) return;

    //     // 「この部位Collider × この障害物Collider」は接触中か？（離れるまで1回）
    //     ulong key = PairKey(self.GetInstanceID(), other.GetInstanceID());
    //     if (!touching.Add(key)) return;

    //     // 自分表面：相手の中心に一番近い点
    //     Vector3 pCar = self.ClosestPoint(other.bounds.center);
    //     // 相手表面：自分の中心に一番近い点
    //     Vector3 pObs = other.ClosestPoint(self.bounds.center);

    //     // 車にくっつく点（部位に追従）
    //     Spawn(dotPrefab, PushOut(pCar, pCar - pObs), self.transform);

    //     // 障害物側の点（ワールド固定：parent=null）
    //     Spawn(dotPrefab, PushOut(pObs, pObs - pCar), null);

    // }

    internal void Exit(Collider self, Collider other)
    {
        if (((1 << other.gameObject.layer) & obstacleMask.value) == 0) return;
        touching.Remove(PairKey(self.GetInstanceID(), other.GetInstanceID()));
    }

    Vector3 PushOut(Vector3 pos, Vector3 outward)
    {
        Vector3 n = outward.sqrMagnitude > 1e-8f ? outward.normalized : Vector3.up;
        return pos + n * offset;
    }

    void Spawn(GameObject prefab, Vector3 pos, Transform parent)
    {
        var go = Instantiate(prefab, pos, Quaternion.identity);
        go.transform.localScale = Vector3.one * dotScale;
        if (parent) go.transform.SetParent(parent, true);
    }

    static ulong PairKey(int a, int b)
    {
        uint ua = (uint)a, ub = (uint)b;
        if (ua > ub) (ua, ub) = (ub, ua);
        return ((ulong)ua << 32) | ub;
    }
}


[RequireComponent(typeof(Collider))]
public sealed class HitMarkerPart : MonoBehaviour
{
    HitMarkerRoot root;
    Collider self;

    // ルート側で1回止まれば十分なので、部位側で多重停止を防ぐ
    static bool stopped;

    SimulationManager sim;

    void Awake()
    {
        root = GetComponentInParent<HitMarkerRoot>();
        self = GetComponent<Collider>();

        // どこかに1個ある前提ならこれが一番手軽（嫌なら root から渡す方式に変えられる）
        sim = FindAnyObjectByType<SimulationManager>();
    }

    void OnTriggerEnter(Collider other)
    {
        if (stopped) return;
        if (root != null && root.TryEnter(self, other))
        {
            stopped = true;
            if (sim != null) sim.StopSimulation();
        }
    }

    void OnTriggerExit(Collider other)
    {
        root?.Exit(self, other);
    }
}
