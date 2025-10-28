using UnityEngine;
using System.Collections;
using System;

[RequireComponent(typeof(Camera))]
public class RoverCameraPublisher : MonoBehaviour
{
    public RosBridgeClient ros;
    
    [Header("Camera Settings")]
    public int imageWidth = 320;   // Smaller resolution for better performance
    public int imageHeight = 240;
    public float publishRate = 5f;  // 5 FPS
    
    private Camera roverCamera;
    private RenderTexture renderTexture;
    private Texture2D imageTexture;
    private bool cameraAdvertised = false;
    
    void Start()
    {
        roverCamera = GetComponent<Camera>();
        
        // Create render texture
        renderTexture = new RenderTexture(imageWidth, imageHeight, 16);
        roverCamera.targetTexture = renderTexture;
        
        // Create texture for reading pixels
        imageTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        
        if (ros != null)
        {
            StartCoroutine(PublishCameraImages());
            Debug.Log("📷 Rover camera publisher ready");
        }
        else
        {
            Debug.LogError("❌ ROS Bridge not assigned to camera!");
        }
    }
    
    IEnumerator PublishCameraImages()
    {
        while (true)
        {
            yield return new WaitUntil(() => ros != null && ros.IsConnected);
            
            if (!cameraAdvertised)
            {
                ros.AdvertiseTopic("/rover_camera/image_raw", "sensor_msgs/Image");
                cameraAdvertised = true;
                Debug.Log("📡 Camera topic advertised");
            }
            
            // Capture and publish image
            PublishCameraImage();
            
            yield return new WaitForSeconds(1f / publishRate);
        }
    }
    
    void PublishCameraImage()
    {
        try
        {
            // Render camera to texture
            RenderTexture.active = renderTexture;
            roverCamera.Render();
            
            // Read pixels from render texture
            imageTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
            imageTexture.Apply();
            
            // Convert to byte array (simplified - using JPEG encoding)
            byte[] imageData = imageTexture.EncodeToJPG(50); // 50% quality for smaller size
            
            // Convert to base64 for ROS transmission
            string base64Image = Convert.ToBase64String(imageData);
            
            var imageMsg = new
            {
                header = new { 
                    stamp = new { sec = (int)Time.time, nanosec = (int)((Time.time % 1) * 1e9) },
                    frame_id = "rover_camera" 
                },
                height = imageHeight,
                width = imageWidth,
                encoding = "jpeg",  // Changed to JPEG
                is_bigendian = false,
                step = imageWidth * 3,
                data = base64Image
            };
            
            ros.Publish("/rover_camera/image_raw", imageMsg);
            
            RenderTexture.active = null;
            
            // Debug log (throttled)
            if (Time.frameCount % 300 == 0) // Every 5 seconds at 60 FPS
            {
                Debug.Log($"📷 Camera published: {imageData.Length} bytes");
            }
        }
        catch (Exception e)
        {
            Debug.LogError("❌ Camera publishing error: " + e.Message);
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
