// Add this script to your camera GameObject
using UnityEngine;

public class RoverCameraLight : MonoBehaviour 
{
    public Light spotlight;
    
    void Start()
    {
        // Create spotlight if not assigned
        if (spotlight == null)
        {
            GameObject lightObj = new GameObject("Camera_Spotlight");
            lightObj.transform.SetParent(this.transform);
            lightObj.transform.localPosition = Vector3.zero;
            lightObj.transform.localRotation = Quaternion.identity;
            
            spotlight = lightObj.AddComponent<Light>();
        }
        
        // Configure spotlight for lunar rock detection
        spotlight.type = LightType.Spot;
        spotlight.color = Color.white;
        spotlight.intensity = 2.5f;        // Bright enough to illuminate rocks
        spotlight.range = 25f;             // Good range for obstacle detection
        spotlight.spotAngle = 45f;         // Wide enough cone
        spotlight.innerSpotAngle = 30f;    // Smooth falloff
        spotlight.shadows = LightShadows.Soft;  // Creates good rock definition
        
        Debug.Log("🔦 Camera spotlight configured for rock detection!");
    }
    
    void Update()
    {
        // Optional: Adjust intensity based on conditions
        if (Input.GetKey(KeyCode.L))
        {
            spotlight.intensity = Mathf.Clamp(spotlight.intensity + 0.1f, 0.5f, 5.0f);
            Debug.Log($"Light intensity: {spotlight.intensity:F1}");
        }
        else if (Input.GetKey(KeyCode.K))
        {
            spotlight.intensity = Mathf.Clamp(spotlight.intensity - 0.1f, 0.5f, 5.0f);
            Debug.Log($"Light intensity: {spotlight.intensity:F1}");
        }
    }
}
