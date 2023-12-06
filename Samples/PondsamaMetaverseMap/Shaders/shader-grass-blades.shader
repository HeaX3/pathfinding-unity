Shader "Unlit/shader-grass-blades"
{
    Properties
    {
        _BaseColor("Base Color", Color) = (1, 1, 1, 1)
        _TipColor("Tip Color", Color) = (1, 1, 1, 1)

        _BladeWidthMin("Blade Width (Min)", Range(0, 1)) = 0.02
        _BladeWidthMax("Blade Width (Max)", Range(0, 1)) = 0.05
        _BladeHeightMin("Blade Height (Min)", Range(0, 2)) = 0.1
        _BladeHeightMax("Blade Height (Max)", Range(0, 2)) = 0.2

        _BladeBendDistance("Blade Forward Amount", Float) = 0.38
        _BladeBendCurve("Blade Curvature Amount", Range(1, 4)) = 2

        _BendDelta("Bend Variation", Range(0, 1)) = 0.2

        _TessellationEdgeLength ("Tessellation Edge Length", Range(5, 100)) = 50

        _GrassThreshold("Grass Visibility Threshold", Range(-0.1, 1)) = 0.5
    }

    HLSLINCLUDE
    #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
    #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

    #define UNITY_PI 3.14159265359f
    #define UNITY_TWO_PI 6.28318530718f
    #define BLADE_SEGMENTS 2

    CBUFFER_START(UnityPerMaterial)
    float4 _BaseColor;
    float4 _TipColor;

    float _BladeWidthMin;
    float _BladeWidthMax;
    float _BladeHeightMin;
    float _BladeHeightMax;

    float _BladeBendDistance;
    float _BladeBendCurve;

    float _BendDelta;

    float _TessellationEdgeLength;

    float _GrassThreshold;

    float4 _ShadowColor;
    CBUFFER_END

    struct VertexInput
    {
        float4 vertex : POSITION;
        float3 normal : NORMAL;
        float4 tangent : TANGENT;
        float4 color : COLOR;
    };

    struct VertexOutput
    {
        float4 vertex : SV_POSITION;
        float3 normal : NORMAL;
        float4 tangent : TANGENT;
        float4 color : COLOR;
    };

    struct GeomData
    {
        float4 pos : SV_POSITION;
        float2 uv : TEXCOORD0;
        float3 worldPos : TEXCOORD1;
    };

    struct TessellationFactors
    {
        float edge[3] : SV_TessFactor;
        float inside : SV_InsideTessFactor;
    };
    ENDHLSL

    SubShader
    {
        Tags
        {
            "RenderType" = "Transpare"
            "Queue" = "Geometry"
            "RenderPipeline" = "UniversalPipeline"
        }
        LOD 100
        Cull Off

        Pass
        {
            Tags
            {
                "LightMode"="UniversalForward"
            }
            HLSLPROGRAM
            #pragma shader_feature _TESSELLATION_EDGE
            #pragma require tessellation tessHW
            #pragma hull hull
            #pragma domain domain
            #pragma require geometry
            #pragma geometry geom
            #pragma vertex geomVert
            #pragma fragment frag

            GeomData TransformGeomToClip(float3 pos, float3 offset, float3x3 transformationMatrix, float2 uv)
            {
                GeomData o;

                o.pos = TransformObjectToHClip(pos + mul(transformationMatrix, offset));
                o.uv = uv;
                o.worldPos = TransformObjectToWorld(pos + mul(transformationMatrix, offset));

                return o;
            }

            float rand(float3 co)
            {
                return frac(sin(dot(co.xyz, float3(12.9898, 78.233, 53.539))) * 43758.5453);
            }

            float3x3 angleAxis3x3(float angle, float3 axis)
            {
                float c, s;
                sincos(angle, s, c);

                float t = 1 - c;
                float x = axis.x;
                float y = axis.y;
                float z = axis.z;

                return float3x3
                (
                    t * x * x + c, t * x * y - s * z, t * x * z + s * y,
                    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
                    t * x * z - s * y, t * y * z + s * x, t * z * z + c
                );
            }

            float TessellationEdgeFactor(float3 p0, float3 p1)
            {
                float edgeLength = distance(p0, p1);
                return edgeLength /
                    (_TessellationEdgeLength);
            }

            TessellationFactors patchConstantFunc(InputPatch<VertexInput, 3> patch)
            {
                float3 p0 = mul(unity_ObjectToWorld, patch[0].vertex).xyz;
                float3 p1 = mul(unity_ObjectToWorld, patch[1].vertex).xyz;
                float3 p2 = mul(unity_ObjectToWorld, patch[2].vertex).xyz;
                TessellationFactors f;
                f.edge[0] = TessellationEdgeFactor(p1, p2);
                f.edge[1] = TessellationEdgeFactor(p2, p0);
                f.edge[2] = TessellationEdgeFactor(p0, p1);
                f.inside =
                (TessellationEdgeFactor(p1, p2) +
                    TessellationEdgeFactor(p2, p0) +
                    TessellationEdgeFactor(p0, p1)) * (1 / 3.0);
                return f;
            }

            VertexOutput tessVert(const VertexInput v)
            {
                VertexOutput o;
                o.vertex = v.vertex;
                o.normal = v.normal;
                o.tangent = v.tangent;
                o.color = v.color;
                return o;
            }


            [domain("tri")]
            [outputcontrolpoints(3)]
            [outputtopology("triangle_cw")]
            [partitioning("integer")]
            [patchconstantfunc("patchConstantFunc")]
            VertexInput hull(InputPatch<VertexInput, 3> patch, uint id : SV_OutputControlPointID)
            {
                return patch[id];
            }


            [domain("tri")]
            VertexOutput domain(TessellationFactors factors, OutputPatch<VertexInput, 3> patch,
                                float3 barycentricCoordinates : SV_DomainLocation)
            {
                VertexInput i;

                // Create interpolation macro.
                #define INTERPOLATE(fieldname) i.fieldname = \
          patch[0].fieldname * barycentricCoordinates.x + \
          patch[1].fieldname * barycentricCoordinates.y + \
          patch[2].fieldname * barycentricCoordinates.z;

                INTERPOLATE(vertex)
                INTERPOLATE(normal)
                INTERPOLATE(tangent)
                INTERPOLATE(color)

                return tessVert(i);
            }


            [maxvertexcount(BLADE_SEGMENTS * 2 + 1)]
            void geom(point VertexOutput input[1], inout TriangleStream<GeomData> triStream)
            {
                float3 worldPos = mul(unity_ObjectToWorld, float4(input[0].vertex.xyz, 1)).xyz;
                float viewDistance = distance(worldPos, _WorldSpaceCameraPos);
                float size = input[0].color.g;
                if (viewDistance < 100 && size >= _GrassThreshold)
                {
                    VertexNormalInputs normalInputs = GetVertexNormalInputs(input[0].normal, input[0].tangent);
                    float3 pos = input[0].vertex.xyz + float3(rand(input[0].vertex.xyz) * 0.5, 0, rand(input[0].vertex.xyz) * 0.5);
                    float3 normal = input[0].normal;
                    float3 tangent = input[0].tangent;
                    float3 bitangent = normalInputs.bitangentWS;

                    float3x3 tangentToLocal = float3x3
                    (
                        tangent.x, bitangent.x, normal.x,
                        tangent.y, bitangent.y, normal.y,
                        tangent.z, bitangent.z, normal.z
                    );


                    // Rotate around the y-axis a random amount.
                    float3x3 randRotMatrix = angleAxis3x3(rand(pos) * UNITY_TWO_PI, float3(0, 0, 1.0f));

                    // Rotate around the bottom of the blade a random amount.
                    float3x3 randBendMatrix = angleAxis3x3(rand(pos.zzx) * _BendDelta * UNITY_PI * 0.5f,
                                                           float3(-1.0f, 0, 0));

                    // Transform the grass blades to the correct tangent space.
                    float3x3 baseTransformationMatrix = mul(tangentToLocal, randRotMatrix);
                    float3x3 tipTransformationMatrix = mul(mul(tangentToLocal, randBendMatrix), randRotMatrix);

                    float width = mul(lerp(_BladeWidthMin, _BladeWidthMax, rand(pos.xzy)), size);
                    float height = mul(lerp(_BladeHeightMin, _BladeHeightMax, rand(pos.zyx)), size);
                    float forward = rand(pos.yyz) * _BladeBendDistance;


                    // Create blade segments by adding two vertices at once.
                    for (int i = 0; i < BLADE_SEGMENTS; ++i)
                    {
                        float t = i / (float)BLADE_SEGMENTS;
                        float3 offset = float3(width * (1 - t), pow(t, _BladeBendCurve) * forward, height * t);

                        float3x3 transformationMatrix = (i == 0) ? baseTransformationMatrix : tipTransformationMatrix;

                        triStream.Append(TransformGeomToClip(pos, float3(offset.x, offset.y, offset.z),
                                                             transformationMatrix, float2(0, t)));
                        triStream.Append(TransformGeomToClip(pos, float3(-offset.x, offset.y, offset.z),
                                                             transformationMatrix, float2(1, t)));
                    }

                    // Add the final vertex at the tip of the grass blade.
                    triStream.Append(TransformGeomToClip(pos, float3(0, forward, height), tipTransformationMatrix,
                                                         float2(0.5, 1)));


                    triStream.RestartStrip();
                }
            }


            VertexOutput geomVert(VertexInput v)
            {
                VertexOutput o;
                o.vertex = float4(TransformObjectToWorld(v.vertex), 1.0f);
                o.normal = TransformObjectToWorldNormal(v.normal);
                o.tangent = v.tangent;
                o.color = v.color;
                return o;
            }


            float4 frag(GeomData i) : SV_Target
            {
                return lerp(_BaseColor, _TipColor, i.uv.y);
            }
            ENDHLSL
        }
    }

}