using System;
using UnityEngine;


public class RungeKutta
{

    private int DIM;
    private Func<float[], float>[] funcs;

    private float[][] k;
    private float[][] q;
    private float[][] r;
    private float[][] x_tmp;

    public float[] x_old;
    public float[] x_new;

    // コントラスタ
    public RungeKutta(int dim, Func<float[], float>[] funcs)
    {
        DIM = dim;
        
        this.funcs = funcs;

        // 状態変数の数 + t なので DIM + 1
        x_old = new float[DIM + 1];
        x_new = new float[DIM + 1];

        k = New(DIM + 1, 4);
        q = New(DIM + 1, 4);
        r = New(DIM + 1, 4);
        x_tmp = New(3, DIM + 1);
    }

    // ルンゲクッタ法による状態更新
    public void UpdateStateRungeKutta(float dt)
    {

        for (int i = 0; i < DIM + 1; i++) {
            k[i][0] = dt * (funcs[i])(x_old);
            r[i][0] = (k[i][0] - 2.0f* q[i][3]) / 2.0f;

            x_tmp[0][i] = x_old[i] + r[i][0];
            q[i][0] = q[i][3] + 3.0f * r[i][0] - k[i][0] / 2.0f;
        }

        for (int i = 0; i < DIM + 1; i++) {
            k[i][1] = dt * (funcs[i])(x_tmp[0]);
            r[i][1] = (1.0f - Mathf.Sqrt(0.5f)) * (k[i][1] - q[i][0]);
            x_tmp[1][i] = x_tmp[0][i] + r[i][1];
            q[i][1] = q[i][0] + 3.0f * r[i][1] - (1.0f - Mathf.Sqrt(0.5f)) * k[i][1];
        }

        for (int i = 0; i < DIM + 1; i++) {
            k[i][2] = dt * (funcs[i])(x_tmp[1]);
            r[i][2] = (1.0f + Mathf.Sqrt(0.5f)) * (k[i][2] - q[i][1]);
            x_tmp[2][i] = x_tmp[1][i] + r[i][2];
            q[i][2] = q[i][1] + 3.0f * r[i][2] - (1.0f + Mathf.Sqrt(0.5f)) * k[i][2];
        }

        for (int i = 0; i < DIM + 1; i++) {
            k[i][3] = dt * (funcs[i])(x_tmp[2]);
            r[i][3] = (k[i][3] - 2.0f * q[i][2]) / 6.0f;
            x_new[i] = x_tmp[2][i] + r[i][3];
            q[i][3] = q[i][2] + 3.0f * r[i][3] - k[i][3] / 2.0f;
        }
    }

    // 次のデータをコミット
    public void CommitStep()
    {
        Array.Copy(x_new, x_old, DIM + 1);
    }

    private static float[][] New(int rows, int cols) {
        var a = new float[rows][];
        for (int i = 0; i < rows; i++) a[i] = new float[cols];
        return a;
    }


}
