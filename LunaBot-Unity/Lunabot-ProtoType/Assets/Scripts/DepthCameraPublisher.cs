using UnityEngine;
using System;
using System.Collections;

[RequireComponent(typeof(Camera))]
public class DepthCameraPublisher : MonoBehaviour
{
    public RosBridgeClient ros;

    public int imageWidth = 320;   // /Tuing: lower = faster
    public int imageHeight = 240;  // /Tuing: lower = faster

    public float publishRate = 5f; // /Tuing: reduce if bandwidth is low

    public float maxDepthRange = 50f; // /Tuing: adjust for scene size

    public Material depthMaterial;
    public bool enableDepthPreview = false;

    private Camera cam;
    private RenderTexture depthRT;
    private RenderTexture rgbRT;
    private Texture2D depthTexture;
    private Texture2D rgbTexture;
    private Texture2D depthColorized;
    private int frameCount = 0;
    private bool isPublishing = false;
    private byte[] depthBytesBuffer;
    private float nextPublishTime = 0f;

    void Start()
    {
        cam = GetComponent<Camera>();
        cam.depthTextureMode = DepthTextureMode.Depth;

        depthRT = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.RFloat);
        depthRT.filterMode = FilterMode.Point;
        depthRT.Create();

        rgbRT = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.RGB565);
        rgbRT.filterMode = FilterMode.Point;
        rgbRT.Create();

        depthTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RFloat, false);
        rgbTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        depthColorized = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        depthBytesBuffer = new byte[imageWidth * imageHeight * 4];

        if (depthMaterial == null)
        {
            Debug.LogError("Depth material missing.");
            enabled = false;
            return;
        }

        if (ros == null)
        {
            Debug.LogError("ROS Bridge missing.");
            return;
        }

        StartCoroutine(Initialize());
    }

    IEnumerator Initialize()
    {
        yield return new WaitUntil(() => ros != null && ros.IsConnected);

        ros.AdvertiseTopic("/camera/color/image_raw", "sensor_msgs/Image");
        ros.AdvertiseTopic("/camera/depth/image_raw", "sensor_msgs/Image");
        ros.AdvertiseTopic("/camera/color/camera_info", "sensor_msgs/CameraInfo");
        ros.AdvertiseTopic("/camera/depth/camera_info", "sensor_msgs/CameraInfo");

        isPublishing = true;
        nextPublishTime = Time.realtimeSinceStartup;
    }

    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        if (isPublishing && Time.realtimeSinceStartup >= nextPublishTime)
        {
            depthMaterial.SetFloat("_MaxDepth", maxDepthRange);
            Graphics.Blit(source, depthRT, depthMaterial);
            Graphics.Blit(source, rgbRT);

            StartCoroutine(CaptureAndPublishAsync());

            nextPublishTime = Time.realtimeSinceStartup + (1f / publishRate);
        }

        Graphics.Blit(source, destination);
    }

    IEnumerator CaptureAndPublishAsync()
    {
        double rosTime = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
        int secs = (int)rosTime;
        int nsecs = (int)((rosTime % 1) * 1e9);

        RenderTexture.active = rgbRT;
        rgbTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0, false);
        rgbTexture.Apply(false, false);
        RenderTexture.active = null;

        Color[] rgbPixels = rgbTexture.GetPixels();
        Color[] rgbFlipped = new Color[rgbPixels.Length];

        // Tuning: this flip is required for ROS image pipeline
        for (int y = 0; y < imageHeight; y++)
            for (int x = 0; x < imageWidth; x++)
                rgbFlipped[x + y * imageWidth] =
                    rgbPixels[x + (imageHeight - 1 - y) * imageWidth];

        rgbTexture.SetPixels(rgbFlipped);
        rgbTexture.Apply();

        byte[] rgbBytes = rgbTexture.GetRawTextureData();

        RenderTexture.active = depthRT;
        depthTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0, false);
        depthTexture.Apply(false, false);
        RenderTexture.active = null;

        Color[] depthPixels = depthTexture.GetPixels();
        Color[] depthFlipped = new Color[depthPixels.Length];

        // Tuning: must flip depth for ROS compatibility
        for (int y = 0; y < imageHeight; y++)
            for (int x = 0; x < imageWidth; x++)
                depthFlipped[x + y * imageWidth] =
                    depthPixels[x + (imageHeight - 1 - y) * imageWidth];

        depthTexture.SetPixels(depthFlipped);
        depthTexture.Apply();

        ColorizeDepth();

        float[] depthValues = new float[depthFlipped.Length];
        for (int i = 0; i < depthFlipped.Length; i++)
            depthValues[i] = depthFlipped[i].r;

        Buffer.BlockCopy(depthValues, 0, depthBytesBuffer, 0, depthBytesBuffer.Length);

        yield return null;

        string base64RGB = Convert.ToBase64String(rgbBytes);
        string base64Depth = Convert.ToBase64String(depthBytesBuffer);

        var rgbMsg = new
        {
            header = new { seq = frameCount, stamp = new { secs, nsecs }, frame_id = "camera_color_optical_frame" },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            encoding = "rgb8",
            is_bigendian = (byte)0,
            step = (uint)(imageWidth * 3),
            data = base64RGB
        };
        ros.Publish("/camera/color/image_raw", rgbMsg);

        var depthMsg = new
        {
            header = new { seq = frameCount, stamp = new { secs, nsecs }, frame_id = "camera_depth_optical_frame" },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            encoding = "32FC1",
            is_bigendian = (byte)0,
            step = (uint)(imageWidth * 4),
            data = base64Depth
        };
        ros.Publish("/camera/depth/image_raw", depthMsg);

        PublishCameraInfo(secs, nsecs);

        frameCount++;
    }

    void ColorizeDepth()
    {
        Color[] pixels = depthTexture.GetPixels();
        Color[] colorized = new Color[pixels.Length];

        for (int i = 0; i < pixels.Length; i++)
        {
            float depth = pixels[i].r;
            float normalized = Mathf.Clamp01(depth / maxDepthRange);

            float r = Mathf.Clamp01(1.5f - Mathf.Abs(normalized * 4f - 3f));
            float g = Mathf.Clamp01(1.5f - Mathf.Abs(normalized * 4f - 2f));
            float b = Mathf.Clamp01(1.5f - Mathf.Abs(normalized * 4f - 1f));

            colorized[i] = new Color(r, g, b);
        }

        Color[] flipped = new Color[colorized.Length];
        for (int y = 0; y < imageHeight; y++)
            for (int x = 0; x < imageWidth; x++)
                flipped[x + y * imageWidth] = colorized[x + (imageHeight - 1 - y) * imageWidth];

        depthColorized.SetPixels(flipped);
        depthColorized.Apply();
    }

    void OnGUI()
    {
        if (enableDepthPreview && Application.isPlaying)
        {
            // Adjust X/Y if your live preview position differs.
            int previewX = 240; // /Tuing: adjust if your live preview size/position changed
            int previewY = 10;
            int previewW = 200;
            int previewH = 120;

            GUI.Box(new Rect(previewX, previewY, previewW + 10, previewH + 30), "");

            if (depthColorized != null)
            {
                GUI.DrawTexture(new Rect(previewX + 10, previewY + 25, previewW, previewH - 30), depthColorized);
            }

            GUI.Label(new Rect(previewX + 10, previewY + 5, 200, 20), $"Depth Frames: {frameCount}");
            string status = isPublishing ? "STREAMING" : "STOPPED";
            GUI.Label(new Rect(previewX + 110, previewY + 5, 120, 20), status);
        }
    }

    void PublishCameraInfo(int secs, int nsecs)
    {
        float fovRad = cam.fieldOfView * Mathf.Deg2Rad;

        // Tuning: smaller resolution = smaller focal length
        float fy = imageHeight / (2f * Mathf.Tan(fovRad / 2f));
        float fx = fy;

        float cx = imageWidth * 0.5f;
        float cy = imageHeight * 0.5f;

        var info = new
        {
            header = new { seq = frameCount, stamp = new { secs, nsecs }, frame_id = "camera_color_optical_frame" },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            distortion_model = "plumb_bob",
            D = new double[5],
            K = new double[] { fx, 0, cx, 0, fy, cy, 0, 0, 1 },
            R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
            P = new double[] { fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0 }
        };
        ros.Publish("/camera/color/camera_info", info);

        var depthInfo = new
        {
            header = new { seq = frameCount, stamp = new { secs, nsecs }, frame_id = "camera_depth_optical_frame" },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            distortion_model = "plumb_bob",
            D = new double[5],
            K = new double[] { fx, 0, cx, 0, fy, cy, 0, 0, 1 },
            R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
            P = new double[] { fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0 }
        };
        ros.Publish("/camera/depth/camera_info", depthInfo);
    }

    void OnDestroy()
    {
        isPublishing = false;
        if (depthRT != null) depthRT.Release();
        if (rgbRT != null) rgbRT.Release();
        if (depthTexture != null) Destroy(depthTexture);
        if (rgbTexture != null) Destroy(rgbTexture);
        if (depthColorized != null) Destroy(depthColorized);
    }
}
