struct Input {
    float2 Position : TEXCOORD0;
    float3 Color : TEXCOORD1;
};

struct Output {
    float4 Color : TEXCOORD0;
    float4 Position : SV_Position;
};

Output main(Input input) {
    Output output;
    output.Color = float4(input.Color, 1.0f);
    output.Position = float4(input.Position, 0.0f, 1.0f);
    return output;
}