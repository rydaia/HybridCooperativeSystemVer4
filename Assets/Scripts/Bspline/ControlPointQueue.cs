using UnityEngine;
using Unity.Profiling;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;

public class ControlPointQueue
{

    // プロファイルマーカー
    static ProfilerMarker pastJobMarker = new ProfilerMarker("CPQueue.PastJob");
    static ProfilerMarker futureJobMarker = new ProfilerMarker("CPQueue.FutureJob");
    static ProfilerMarker mergeJobMarker = new ProfilerMarker("CPQueue.MergeJob");

    // 過去
    NativeArray<float2> past;      // 過去キュー（リングバッファ）
    int pastHead;                  // FIFO head index（取り出し位置）
    int pastCount;                 // 現在の要素数（np 固定）
    int Np;

    // 未来
    NativeArray<float2> future;    // 未来予測 (Nf)
    int Nf;

    // マージ後の制御点集合
    NativeArray<float2> merged;    // Np + Nf

    private float dsTotal = 0f;
    private float dt;      // シミュレーション刻み幅

    private TargetPointState target;

    // コントラスタ
    public ControlPointQueue()
    {
    }

    public void Initialize(int Np, int Nf, float dt, CalculationManager calc)
    {

        target = calc.targetPointState;
        this.Np = Np;
        this.Nf = Nf;
        this.dt = dt;

        past     = new NativeArray<float2>(Np, Allocator.Persistent);
        future   = new NativeArray<float2>(Nf, Allocator.Persistent);
        merged   = new NativeArray<float2>(Np + Nf, Allocator.Persistent);

        pastHead = 0;
        pastCount = 0;

        // / --- 初回の過去・未来データ生成 ---
        Vector2 Tp0 = calc.targetPointState.GetPosition();
        float theta0 = calc.targetPointState.getTheta();
        float kappa0 = calc.targetPointState.getKappa();

        InitializePastQueue(Tp0, theta0);
        InitializeFuturePoints(Tp0, theta0, kappa0);

        Merge();
        Debug.Log($"InitializeMergeData complete. Length={merged.Length}");

    }

    public void Update( Vector2 Tp, float theta, float v1, float kappa )
    {

        // 目標点が止まっている時は過去の軌跡データの更新はしない
        using (pastJobMarker.Auto())
        {
            if(v1 > 0.0f) UpdatePastQueue(Tp, v1);
        }

        using (futureJobMarker.Auto())
        {
            UpdateFuturePoints(Tp, theta, kappa);
        }

        using (mergeJobMarker.Auto())
        {
            // 5. 過去の軌跡データ + 未来の予測軌道データ のマージ
            Merge();
        }
    }



    // TPの初期値からnp分姿勢角の荷台方向へds刻みで格納していく
    // 過去 NP 個
    // 初期位置TP から後方へ ds 刻みで並べる
    // 逆にする
    public void InitializePastQueue(Vector2 Tp, float theta)
    {
        pastHead = 0;
        pastCount = 0;

        // 過去データの初期化時は距離0.01間隔
        float dp = 0.01f;

        Debug.Log($"Tp={Tp}");


        float2 d = new float2(math.cos(theta), math.sin(theta));

        float2 start = new float2(
            Tp.x - d.x * dt * (Np - 1),
            Tp.y - d.y * dt * (Np - 1)
        );

        Debug.Log($"start={start}");


        for (int i = 0; i < Np; i++)
        {
            float2 p = start + d * (dp * i);
            EnqueuePast(p);
        }

        Debug.Log($"InitializePastQueue complete. Count={pastCount}");
    }

    void UpdatePastQueue(Vector2 Tp, float v1)
    {

        const float V_THRESHOLD = 0.05f; // 停止判定

        if (v1 < V_THRESHOLD)
        {
            // 停止中：過去点を追加しない → カーブ安定
            return;
        }

        float2 prev = past[(pastHead + pastCount - 1) % Np];
        float dist = math.distance(prev, Tp);

        float addThreshold = 0.01f * 0.5f; // dPast * 0.5 = 0.005m

        // 一定以上動いたら追加（密集防止）
        if (dist > addThreshold) // しきい値（調整可）
        {
            // 追加
            EnqueuePast(Tp);

            // 総距離超過チェック
            while (GetTotalPastLength() > Np * 0.01f)
                DequeuePast(1);
        }

        // 前のモードがSTopで今がForawrdの時、更新はしない
        // 現在の座標がpastの後ろに追加され、同じ座標が二つ入ってしまうため
        // if(target.GetPrevMode() == TargetPointMode.Stop) return;

        // EnqueuePast(Tp);
    }

    float GetTotalPastLength()
    {
        float sum = 0f;
        for (int i = 1; i < pastCount; i++)
        {
            int idx0 = (pastHead + i - 1) % Np;
            int idx1 = (pastHead + i) % Np;
            sum += math.distance(past[idx0], past[idx1]);
        }
        return sum;
    }

    // NativeArray FIFO (enqueue)
    void EnqueuePast(float2 v)
    {
        if (pastCount < Np)
        {
            int tail = (pastHead + pastCount) % Np;
            past[tail] = v;
            pastCount++;
        }
        else
        {
            // 上書き（最大サイズ時）
            past[pastHead] = v;
            pastHead = (pastHead + 1) % Np;
        }
    }

    // dequeue n 個
    void DequeuePast(int n)
    {
        n = math.min(n, pastCount);
        pastHead = (pastHead + n) % Np;
        pastCount -= n;
    }


    // TPの初期値からnf分姿勢角(theta)方向へへds刻みで格納していく
    // 初期位置TP から未来方向 θ に向かってds 間隔で NF 個さらに曲率 κ に沿って θ を更新しながら未来点を生成する
    public void InitializeFuturePoints(Vector2 Tp, float theta, float kappa)
    {
        float2 tp = new float2(Tp.x, Tp.y);

        Debug.Log($"kappa:{kappa}");


        // 未来の軌道データは距離0.01間隔で初期化
        float df = 0.01f;

        for (int k = 0; k < Nf; k++)
        {
            float s = df * (k + 1);
            float theta_k = theta + kappa * s;

            future[k] = new float2(
                tp.x + math.cos(theta_k) * s,
                tp.y + math.sin(theta_k) * s
            );
        }
    }

    void UpdateFuturePoints(Vector2 Tp, float theta, float kappa)
    {
        // 4. 未来予測データを再生成
        var job = new FutureJob
        {
            Tp = new float2(Tp.x, Tp.y),
            theta = theta,
            kappa = kappa,
            d = dt,
            output = future
        };

        JobHandle handle = job.Schedule(Nf, 64);
        handle.Complete();
    }

    public void Merge()
    {
        var job = new MergeJob
        {
            past = past,
            pastHead = pastHead,
            pastCount = pastCount,
            np = Np,
            future = future,
            merged = merged
        };

        job.Schedule(Np + Nf, 64).Complete();

        // for (int i = 0; i < Np + Nf; i++)
        //     Debug.Log($"i:{i}, merged[i]:{merged[i]}");
    }

    public void Dispose()
    {
        if (past.IsCreated) past.Dispose();
        if (future.IsCreated) future.Dispose();
        if (merged.IsCreated) merged.Dispose();
    }

    public int GetNp()
    {
        return Np;
    }

    public int GetNf()
    {
        return Nf;
    }

    public NativeArray<float2> GetMergedNative()
    {
        return merged;
    }

    public float2[] GetMergedPointsManaged()
    {
        int N = Nf + Np;
        float2[] arr = new float2[N];
        for (int i = 0; i < N; i++)
            arr[i] = merged[i];
        return arr;
    }

    public float2[] GetPastPointsManaged()
    {
        float2[] arr = new float2[pastCount];
        for (int i = 0; i < pastCount; i++)
        {
            int idx = (pastHead + i) % Np;
            arr[i] = past[idx];
        }
        return arr;
    }

    public float2[] GetFuturePointsManaged()
    {
        float2[] arr = new float2[Nf];
        for (int i = 0; i < Nf; i++)
            arr[i] = future[i];
        return arr;
    }

}
