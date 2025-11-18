using UnityEngine;
using System;
using System.Collections;

[RequireComponent(typeof(Camera))]
public class LiveCameraPublisher : MonoBehaviour
{
    public RosBridgeClient ros;

    [Header("Live Camera Settings")]
    public int imageWidth = 640;          // /Tuing: Reduce for lower CPU/bandwidth
    public int imageHeight = 480;         // /Tuing: Reduce for lower CPU/bandwidth
    public float publishRate = 15f;       // /Tuing: 5-20Hz typical; lower if CPU constrained
    public int jpegQuality = 75;          // /Tuing: 50-85 for good speed/quality tradeoff

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

        Debug.Log($"Live Camera initialized: {imageWidth}x{imageHeight} @ {publishRate}fps");

        if (ros != null)
            StartCoroutine(InitializeLiveStreaming());
        else
            Debug.LogError("ROS Bridge not assigned to live camera");
    }

    IEnumerator InitializeLiveStreaming()
    {
        yield return new WaitUntil(() => ros != null && ros.IsConnected);

        ros.AdvertiseTopic("/rover_camera/image_raw", "sensor_msgs/Image");
        cameraAdvertised = true;

        yield return new WaitForSeconds(1f);

        isPublishing = true;
        StartCoroutine(LiveVideoStreaming());
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
                // keep light warning, avoid spam
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

            var imageMsg = new
            {
                header = new
                {
                    seq = frameCount,
                    stamp = new
                    {
                        sec = (int)Time.time,
                        nanosec = (int)((Time.time % 1) * 1e9)
                    },
                    frame_id = "rover_camera_live"
                },
                height = (uint)imageHeight,
                width = (uint)imageWidth,
                encoding = "jpeg",
                is_bigendian = (byte)0,
                step = (uint)(imageWidth * 3),
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
                    Debug.Log($"Live stream frame #{frameCount} | Size: {imageData.Length} bytes | avgPublishTime: {avgPublishTime:F3}s");
                }
            }
            catch (Exception publishError)
            {
                Debug.LogWarning($"Failed to publish frame #{frameCount}: {publishError.Message}");
            }

            RenderTexture.active = null;
        }
        catch (Exception e)
        {
            Debug.LogError($"Live camera error at frame #{frameCount}: {e.Message}");
        }

        yield return null;
    }

    public void StartStreaming()
    {
        if (!isPublishing && ros != null && ros.IsConnected)
        {
            isPublishing = true;
            StartCoroutine(LiveVideoStreaming());
        }
    }

    public void StopStreaming()
    {
        isPublishing = false;
    }

    public void SetFrameRate(float fps)
    {
        publishRate = Mathf.Clamp(fps, 1f, 30f); // /Tuing: keep within sensor/ros limits
    }

    public void SetQuality(int quality)
    {
        jpegQuality = Mathf.Clamp(quality, 10, 100); // /Tuing: lower to reduce bandwidth
    }

    public string GetPerformanceStats()
    {
        float actualFps = publishedFrames > 0 && avgPublishTime > 0 ? 1f / avgPublishTime : 0f;
        return $"Frames: {frameCount} | Target: {publishRate}fps | Published: {publishedFrames} | avgPublishFPS: {actualFps:F1}";
    }

    void OnGUI()
    {
        if (enableLivePreview && Application.isPlaying)
        {
            GUI.Box(new Rect(10, 10, 200, 120), "Live Camera Feed");

            if (renderTexture != null)
                GUI.DrawTexture(new Rect(15, 35, 190, 90), renderTexture);

            GUI.Label(new Rect(15, 130, 300, 20), GetPerformanceStats());

            string status = isPublishing ? "STREAMING" : "STOPPED";
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
            renderTexture.Release();
    }

    void OnValidate()
    {
        publishRate = Mathf.Clamp(publishRate, 1f, 30f);
        jpegQuality = Mathf.Clamp(jpegQuality, 10, 100);
        imageWidth = Mathf.Clamp(imageWidth, 160, 1920);
        imageHeight = Mathf.Clamp(imageHeight, 120, 1080);
    }
}
