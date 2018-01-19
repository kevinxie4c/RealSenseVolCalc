cbuffer MatrixBuffer
{
    matrix transform;
};

struct VOut
{
    float4 position: SV_POSITION;
    float4 color: COLOR;
    float3 normal: NORMAL;
};

VOut VShader(float4 position: POSITION, float4 color: COLOR, float3 normal: NORMAL)
{
    VOut output;
    position.w = 1.0f;
    output.position = mul(position, transform);
    output.color = color;
    output.normal = normal;
    return output;
}


float4 PShader(float4 position: SV_POSITION, float4 color: COLOR, float3 normal: NORMAL): SV_TARGET
{
	float3 lightDir = {-1, -1, 1};
	float3 v = normalize(-lightDir);
    return color * dot(normal, v);
}