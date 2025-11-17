Shader "Custom/DepthToFloat"
{
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            
            sampler2D _CameraDepthTexture;
            float _MaxDepth;
            
            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };
            
            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 scrPos : TEXCOORD0;
            };
            
            v2f vert(appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.scrPos = ComputeScreenPos(o.pos);
                return o;
            }
            
            float4 frag(v2f i) : SV_Target
            {
                // Sample depth buffer
                float depth = SAMPLE_DEPTH_TEXTURE_PROJ(_CameraDepthTexture, UNITY_PROJ_COORD(i.scrPos));
                
                // Convert to linear depth in meters
                float linearDepth = LinearEyeDepth(depth);
                
                // Store in R channel (32-bit float)
                return float4(linearDepth, 0, 0, 1);
            }
            ENDCG
        }
    }
}
