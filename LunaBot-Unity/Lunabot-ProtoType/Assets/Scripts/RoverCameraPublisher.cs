using UnityEngine;
using System.Collections;
using System;

[RequireComponent(typeof(Camera))]
public class RoverCameraPublisher : MonoBehaviour
{
    public RosBridgeClient ros;

    [Header("Camera Settings")]
    public int imageWidth = 320;   // /Tuing: reduce for CPU/network
    public int imageHeight = 240;  // /Tuing: reduce for CPU/network
    public float publishRate = 5f; // /Tuing: target FPS for bandwidth/control
    public int jpegQuality = 50;   // /Tuing: lower = smaller packets

    private Camera roverCamera;
    private RenderTexture renderTexture;
    private Texture2D imageTexture;
    private bool cameraAdvertised = false;

    void Start()
    {
        roverCamera = GetComponent<Camera>();

        renderTexture = new RenderTexture(imageWidth, imageHeight, 16);
        roverCamera.targetTexture = renderTexture;

        imageTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        if (ros != null)
        {
            StartCoroutine(PublishCameraImages());
        }
        else
        {
            Debug.LogError("ROS Bridge not assigned to camera");
        }
    }

    IEnumerator PublishCameraImages()
    {
        var waitForRate = new WaitForSeconds(1f / Mathf.Max(0.001f, publishRate));

        while (true)
        {
            yield return new WaitUntil(() => ros != null && ros.IsConnected);

            if (!cameraAdvertised)
            {
                ros.AdvertiseTopic("/rover_camera/image_raw", "sensor_msgs/Image");
                cameraAdvertised = true;
            }

            PublishCameraImage();

            yield return waitForRate;
        }
    }

    void PublishCameraImage()
    {
        try
        {
            RenderTexture.active = renderTexture;
            roverCamera.Render();

            imageTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
            imageTexture.Apply();

            byte[] imageData = imageTexture.EncodeToJPG(Mathf.Clamp(jpegQuality, 10, 100));
            string base64Image = Convert.ToBase64String(imageData);

            var imageMsg = new
            {
                header = new
                {
                    stamp = new
                    {
                        sec = (int)Time.time,
                        nanosec = (int)((Time.time % 1) * 1e9)
                    },
                    frame_id = "rover_camera"
                },
                height = (uint)imageHeight,
                width = (uint)imageWidth,
                encoding = "jpeg",
                is_bigendian = (byte)0,
                step = (uint)(imageWidth * 3),
                data = base64Image
            };

            ros.Publish("/rover_camera/image_raw", imageMsg);

            RenderTexture.active = null;

            if (Time.frameCount % 300 == 0)
            {
                Debug.Log($"Camera published ({imageData.Length} bytes)");
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Camera publishing error: " + e.Message);
        }
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
        }
    }
}
