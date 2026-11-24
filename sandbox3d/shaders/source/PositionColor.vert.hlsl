struct Input {
    float3 Position : TEXCOORD0;
    float3 Color : TEXCOORD1;
};

cbuffer TransformData : register(b0, space1) {
    column_major float4x4 MVP;
};

struct Output {
    float4 Color : TEXCOORD0;
    float4 Position : SV_Position;
};

Output main(Input input) {
    Output output;

    float4 localPos = float4(input.Position, 1.0f);
    output.Position = mul(MVP, localPos);

    output.Color = float4(input.Color, 1.0f);

    return output;
}