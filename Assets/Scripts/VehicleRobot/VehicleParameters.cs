[System.Serializable]
public class VehicleParameters
{
    public float L1;  // 前軸～ステア軸
    public float L2;  // ステア軸～後輪
    public float L3;  // 車体の全長など

    public VehicleParameters()
    {
        float scale = 5.0f;

        L1 = 0.27f * scale;
        L2 = 0.81f * scale;
        L3 = 0.27f * scale;
    }

    public float GetL1() => L1;
    public float GetL2() => L2;
    public float GetL3() => L3;
}
