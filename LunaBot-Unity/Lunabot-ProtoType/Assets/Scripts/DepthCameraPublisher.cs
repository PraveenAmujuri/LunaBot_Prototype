using UnityEngine;
using System;
using System.Collections;

[RequireComponent(typeof(Camera))]
public class DepthCameraPublisher : MonoBehaviour
{
    [Header("ROSBridge Connection")]
    public RosBridgeClient ros;
    
    [Header("Camera Settings")]
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float publishRate = 10f;
    public float maxDepthRange = 50f;
    
    [Header("Depth Shader")]
    public Material depthMaterial;
    
    [Header("Performance")]
    public bool enableDepthPreview = false;
    
    private Camera cam;
    private RenderTexture depthRT;
    private RenderTexture rgbRT;
    private Texture2D depthTexture;
    private Texture2D rgbTexture;
    private Texture2D depthColorized;
    private bool advertised = false;
    private int frameCount = 0;
    private float lastPublishTime = 0f;
    private bool isPublishing = false;
    
    void Start()
    {
        cam = GetComponent<Camera>();
        cam.depthTextureMode = DepthTextureMode.Depth;
        
        // Depth RT
        depthRT = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.RFloat);
        depthRT.filterMode = FilterMode.Point;
        depthRT.Create();
        
        // RGB RT
        rgbRT = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.RGB565);
        rgbRT.Create();
        
        depthTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RFloat, false);
        rgbTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        depthColorized = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        
        if (depthMaterial == null)
        {
            Debug.LogError("❌ Depth material not assigned!");
            enabled = false;
            return;
        }
        
        Debug.Log($"📏 RGB-D Camera initialized: {imageWidth}x{imageHeight} @ {publishRate}Hz");
        
        if (ros != null)
        {
            StartCoroutine(Initialize());
        }
        else
        {
            Debug.LogError("❌ ROS Bridge not assigned!");
        }
    }
    
    private double GetROSTime()
    {
        return (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
    }
    
    IEnumerator Initialize()
    {
        yield return new WaitUntil(() => ros != null && ros.IsConnected);
        Debug.Log("📏 Advertising RGB-D camera topics...");
        
        ros.AdvertiseTopic("/camera/color/image_raw", "sensor_msgs/Image");
        ros.AdvertiseTopic("/camera/depth/image_raw", "sensor_msgs/Image");
        ros.AdvertiseTopic("/camera/color/camera_info", "sensor_msgs/CameraInfo");
        ros.AdvertiseTopic("/camera/depth/camera_info", "sensor_msgs/CameraInfo");
        
        advertised = true;
        
        yield return new WaitForSeconds(1f);
        
        isPublishing = true;
        StartCoroutine(PublishingLoop());
        
        Debug.Log($"📏 RGB-D streaming started at {publishRate} Hz");
    }
    
    IEnumerator PublishingLoop()
    {
        while (isPublishing)
        {
            if (ros != null && ros.IsConnected && advertised)
            {
                float timeSinceLastPublish = Time.time - lastPublishTime;
                float targetInterval = 1f / publishRate;
                
                if (timeSinceLastPublish >= targetInterval)
                {
                    yield return StartCoroutine(CaptureAndPublish());
                    lastPublishTime = Time.time;
                }
            }
            else
            {
                yield return new WaitForSeconds(1f);
            }
            
            yield return null;
        }
    }
    
    // FIXED: Don't apply depth material to Game view (prevents mirror effect)
    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        // Render depth to internal RT only (not to screen)
        depthMaterial.SetFloat("_MaxDepth", maxDepthRange);
        Graphics.Blit(source, depthRT, depthMaterial);
        
        // Render RGB to internal RT only
        Graphics.Blit(source, rgbRT);
        
        // Pass original source directly to destination (normal view)
        Graphics.Blit(source, destination);
    }
    
    IEnumerator CaptureAndPublish()
    {
        try
        {
            double rosTime = GetROSTime();
            int secs = (int)rosTime;
            int nsecs = (int)((rosTime % 1) * 1e9);
            
            // ═══════════ CAPTURE RGB ═══════════
            RenderTexture.active = rgbRT;
            rgbTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
            rgbTexture.Apply();
            
            // FLIP VERTICALLY for ROS optical frame convention
            Color[] rgbPixels = rgbTexture.GetPixels();
            Color[] flippedRGB = new Color[rgbPixels.Length];
            for (int y = 0; y < imageHeight; y++)
            {
                for (int x = 0; x < imageWidth; x++)
                {
                    flippedRGB[x + y * imageWidth] = rgbPixels[x + (imageHeight - 1 - y) * imageWidth];
                }
            }
            rgbTexture.SetPixels(flippedRGB);
            rgbTexture.Apply();
            
            byte[] rgbBytes = rgbTexture.GetRawTextureData();
            string base64RGB = Convert.ToBase64String(rgbBytes);
            
            var rgbMsg = new
            {
                header = new
                {
                    seq = frameCount,
                    stamp = new { secs = secs, nsecs = nsecs },
                    frame_id = "camera_color_optical_frame"
                },
                height = (uint)imageHeight,
                width = (uint)imageWidth,
                encoding = "rgb8",
                is_bigendian = (byte)0,
                step = (uint)(imageWidth * 3),
                data = base64RGB
            };
            ros.Publish("/camera/color/image_raw", rgbMsg);
            
            // ═══════════ CAPTURE DEPTH ═══════════
            RenderTexture.active = depthRT;
            depthTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
            depthTexture.Apply();
            RenderTexture.active = null;
            
            Color[] depthPixels = depthTexture.GetPixels();
            float[] depthValues = new float[depthPixels.Length];
            
            // FLIP DEPTH VERTICALLY
            for (int y = 0; y < imageHeight; y++)
            {
                for (int x = 0; x < imageWidth; x++)
                {
                    int srcIdx = x + (imageHeight - 1 - y) * imageWidth;
                    int dstIdx = x + y * imageWidth;
                    depthValues[dstIdx] = depthPixels[srcIdx].r;
                }
            }
            
            byte[] depthBytes = new byte[depthValues.Length * 4];
            Buffer.BlockCopy(depthValues, 0, depthBytes, 0, depthBytes.Length);
            string base64Depth = Convert.ToBase64String(depthBytes);
            
            var depthMsg = new
            {
                header = new
                {
                    seq = frameCount,
                    stamp = new { secs = secs, nsecs = nsecs },
                    frame_id = "camera_depth_optical_frame"
                },
                height = (uint)imageHeight,
                width = (uint)imageWidth,
                encoding = "32FC1",
                is_bigendian = (byte)0,
                step = (uint)(imageWidth * 4),
                data = base64Depth
            };
            ros.Publish("/camera/depth/image_raw", depthMsg);
            
            // ═══════════ PUBLISH CAMERA INFO ═══════════
            PublishCameraInfo(secs, nsecs);
            
            frameCount++;
            
            if (frameCount % 30 == 0)
            {
                Debug.Log($"📏 RGB-D frame #{frameCount} | RGB: {rgbBytes.Length}b | Depth: {depthBytes.Length}b");
            }
            
        }
        catch (Exception e)
        {
            Debug.LogError($"❌ RGB-D publish error: {e.Message}");
        }
        
        yield return null;
    }
    
    void PublishCameraInfo(int secs, int nsecs)
    {
        float fovRad = cam.fieldOfView * Mathf.Deg2Rad;
        float fy = imageHeight / (2.0f * Mathf.Tan(fovRad / 2.0f));
        float fx = fy;
        float cx = imageWidth / 2.0f;
        float cy = imageHeight / 2.0f;
        
        var rgbInfo = new
        {
            header = new
            {
                seq = frameCount,
                stamp = new { secs = secs, nsecs = nsecs },
                frame_id = "camera_color_optical_frame"
            },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            distortion_model = "plumb_bob",
            D = new double[5] { 0, 0, 0, 0, 0 },
            K = new double[9] { fx, 0, cx, 0, fy, cy, 0, 0, 1 },
            R = new double[9] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
            P = new double[12] { fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0 }
        };
        ros.Publish("/camera/color/camera_info", rgbInfo);
        
        var depthInfo = new
        {
            header = new
            {
                seq = frameCount,
                stamp = new { secs = secs, nsecs = nsecs },
                frame_id = "camera_depth_optical_frame"
            },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            distortion_model = "plumb_bob",
            D = new double[5] { 0, 0, 0, 0, 0 },
            K = new double[9] { fx, 0, cx, 0, fy, cy, 0, 0, 1 },
            R = new double[9] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
            P = new double[12] { fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0 }
        };
        ros.Publish("/camera/depth/camera_info", depthInfo);
    }
    
    void ColorizeDepth()
    {
        Color[] pixels = depthTexture.GetPixels();
        Color[] colorized = new Color[pixels.Length];
        
        for (int i = 0; i < pixels.Length; i++)
        {
            float depth = pixels[i].r;
            float normalized = Mathf.Clamp01(depth / maxDepthRange);
            
            float r = Mathf.Clamp01(1.5f - Mathf.Abs(normalized * 4 - 3));
            float g = Mathf.Clamp01(1.5f - Mathf.Abs(normalized * 4 - 2));
            float b = Mathf.Clamp01(1.5f - Mathf.Abs(normalized * 4 - 1));
            
            colorized[i] = new Color(r, g, b);
        }
        
        depthColorized.SetPixels(colorized);
        depthColorized.Apply();
    }
    
    void OnGUI()
    {
        if (enableDepthPreview && Application.isPlaying)
        {
            GUI.Box(new Rect(220, 10, 200, 120), "RGB-D Camera");
            
            if (depthRT != null)
            {
                ColorizeDepth();
                GUI.DrawTexture(new Rect(225, 35, 190, 90), depthColorized);
            }
            
            GUI.Label(new Rect(225, 130, 200, 20), $"Frames: {frameCount}");
            
            string status = isPublishing ? "🔴 STREAMING" : "⚫ STOPPED";
            GUI.Label(new Rect(225, 150, 200, 20), status);
        }
    }
    
    void OnDestroy()
    {
        isPublishing = false;
        
        if (depthRT != null) depthRT.Release();
        if (rgbRT != null) rgbRT.Release();
        if (depthTexture != null) Destroy(depthTexture);
        if (rgbTexture != null) Destroy(rgbTexture);
        if (depthColorized != null) Destroy(depthColorized);
        
        Debug.Log($"📏 RGB-D camera destroyed. Total frames: {frameCount}");
    }
}
