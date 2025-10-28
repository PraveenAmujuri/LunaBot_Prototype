using UnityEngine;
using System.Collections;
using System;

[RequireComponent(typeof(Camera))]
public class LiveCameraPublisher : MonoBehaviour
{
    public RosBridgeClient ros;
    
    [Header("Live Camera Settings")]
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float publishRate = 15f;
    public int jpegQuality = 75;
    
    [Header("Performance Settings")]
    public bool enableLivePreview = true;
    public bool optimizeForPerformance = true;
    
    private Camera roverCamera;
    private RenderTexture renderTexture;
    private Texture2D imageTexture;
    private bool cameraAdvertised = false;
    private int frameCount = 0;
    private float lastPublishTime = 0f;
    private bool isPublishing = false;
    
    private float avgPublishTime = 0f;
    private int publishedFrames = 0;
    
    void Start()
    {
        roverCamera = GetComponent<Camera>();
        
        if (optimizeForPerformance)
        {
            roverCamera.allowHDR = false;
            roverCamera.allowMSAA = false;
        }
        
        renderTexture = new RenderTexture(imageWidth, imageHeight, 16, RenderTextureFormat.RGB565);
        renderTexture.filterMode = FilterMode.Bilinear;
        roverCamera.targetTexture = renderTexture;
        
        imageTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        
        Debug.Log($"📹 Live Camera Publisher initialized: {imageWidth}x{imageHeight} @ {publishRate}fps");
        
        if (ros != null)
        {
            StartCoroutine(InitializeLiveStreaming());
        }
        else
        {
            Debug.LogError("❌ ROS Bridge not assigned to live camera!");
        }
    }
    
    IEnumerator InitializeLiveStreaming()
    {
        yield return new WaitUntil(() => ros != null && ros.IsConnected);
        Debug.Log("📹 ROS connected, initializing live video stream...");
        
        ros.AdvertiseTopic("/rover_camera/image_raw", "sensor_msgs/Image");
        cameraAdvertised = true;
        Debug.Log("📡 Live camera topic advertised");
        
        yield return new WaitForSeconds(1f);
        
        isPublishing = true;
        StartCoroutine(LiveVideoStreaming());
        
        Debug.Log($"🎬 Live video streaming started at {publishRate} FPS");
    }
    
    IEnumerator LiveVideoStreaming()
    {
        while (isPublishing)
        {
            if (ros != null && ros.IsConnected && cameraAdvertised)
            {
                float timeSinceLastPublish = Time.time - lastPublishTime;
                float targetInterval = 1f / publishRate;
                
                if (timeSinceLastPublish >= targetInterval)
                {
                    yield return StartCoroutine(CaptureAndPublishFrame());
                    lastPublishTime = Time.time;
                }
            }
            else
            {
                Debug.LogWarning("📹 Waiting for ROS connection...");
                yield return new WaitForSeconds(1f);
            }
            
            yield return null;
        }
    }
    
    IEnumerator CaptureAndPublishFrame()
    {
        float startTime = Time.time;
        
        try
        {
            RenderTexture.active = renderTexture;
            roverCamera.Render();
            
            imageTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0, false);
            imageTexture.Apply();
            
            byte[] imageData = imageTexture.EncodeToJPG(jpegQuality);
            string base64Image = Convert.ToBase64String(imageData);
            
            // FIXED ROS message with correct types
            var imageMsg = new
            {
                header = new { 
                    seq = frameCount,
                    stamp = new { 
                        sec = (int)Time.time, 
                        nanosec = (int)((Time.time % 1) * 1e9) 
                    },
                    frame_id = "rover_camera_live" 
                },
                height = (uint)imageHeight,        // Fixed: uint
                width = (uint)imageWidth,          // Fixed: uint
                encoding = "jpeg",
                is_bigendian = (byte)0,           // Fixed: byte 0 instead of bool false
                step = (uint)(imageWidth * 3),    // Fixed: uint
                data = base64Image
            };
            
            try
            {
                ros.Publish("/rover_camera/image_raw", imageMsg);
                
                frameCount++;
                publishedFrames++;
                
                float publishTime = Time.time - startTime;
                avgPublishTime = (avgPublishTime * (publishedFrames - 1) + publishTime) / publishedFrames;
                
                if (frameCount % 30 == 0)
                {
                    Debug.Log($"📹 Live stream: Frame #{frameCount} | Size: {imageData.Length}b | Target FPS: {publishRate}");
                }
            }
            catch (Exception publishError)
            {
                Debug.LogWarning($"❌ Failed to publish frame #{frameCount}: {publishError.Message}");
            }
            
            RenderTexture.active = null;
            
        }
        catch (Exception e)
        {
            Debug.LogError($"❌ Live camera error at frame #{frameCount}: {e.Message}");
        }
        
        yield return null;
    }
    
    public void StartStreaming()
    {
        if (!isPublishing && ros != null && ros.IsConnected)
        {
            isPublishing = true;
            StartCoroutine(LiveVideoStreaming());
            Debug.Log("🎬 Live streaming resumed");
        }
    }
    
    public void StopStreaming()
    {
        isPublishing = false;
        Debug.Log("⏹️ Live streaming stopped");
    }
    
    public void SetFrameRate(float fps)
    {
        publishRate = Mathf.Clamp(fps, 1f, 30f);
        Debug.Log($"📹 Frame rate set to {publishRate} FPS");
    }
    
    public void SetQuality(int quality)
    {
        jpegQuality = Mathf.Clamp(quality, 10, 100);
        Debug.Log($"📹 JPEG quality set to {jpegQuality}%");
    }
    
    public string GetPerformanceStats()
    {
        float actualFps = publishedFrames > 0 && avgPublishTime > 0 ? 1f / avgPublishTime : 0f;
        return $"Frames: {frameCount} | Target: {publishRate}fps | Published: {publishedFrames}";
    }
    
    void OnGUI()
    {
        if (enableLivePreview && Application.isPlaying)
        {
            GUI.Box(new Rect(10, 10, 200, 120), "Live Camera Feed");
            
            if (renderTexture != null)
            {
                GUI.DrawTexture(new Rect(15, 35, 190, 90), renderTexture);
            }
            
            GUI.Label(new Rect(15, 130, 300, 20), GetPerformanceStats());
            
            string status = isPublishing ? "🔴 STREAMING" : "⚫ STOPPED";
            GUI.Label(new Rect(15, 150, 200, 20), status);
            
            if (GUI.Button(new Rect(15, 170, 80, 25), isPublishing ? "Stop" : "Start"))
            {
                if (isPublishing) StopStreaming();
                else StartStreaming();
            }
            
            if (GUI.Button(new Rect(100, 170, 60, 25), "Low Q"))
            {
                SetQuality(50);
                SetFrameRate(10f);
            }
            
            if (GUI.Button(new Rect(165, 170, 60, 25), "High Q"))
            {
                SetQuality(85);
                SetFrameRate(20f);
            }
        }
    }
    
    void OnDestroy()
    {
        StopStreaming();
        
        if (renderTexture != null)
        {
            renderTexture.Release();
        }
        
        Debug.Log($"📹 Live camera destroyed. Total frames published: {frameCount}");
    }
    
    void OnValidate()
    {
        publishRate = Mathf.Clamp(publishRate, 1f, 30f);
        jpegQuality = Mathf.Clamp(jpegQuality, 10, 100);
        imageWidth = Mathf.Clamp(imageWidth, 160, 1920);
        imageHeight = Mathf.Clamp(imageHeight, 120, 1080);
    }
}
