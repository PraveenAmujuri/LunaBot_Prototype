using UnityEngine;
using System.Collections.Generic;

public class RoverHazardDetector : MonoBehaviour
{
    public RosBridgeClient ros;
    private bool hazardAdvertised = false;
    private HashSet<string> detectedHazards = new HashSet<string>(); // Prevent duplicate alerts

    void Start()
    {
        if (ros == null)
        {
            Debug.LogError("RosBridgeClient is not assigned on RoverHazardDetector!");
            return;
        }
        
        StartCoroutine(AdvertiseAfterConnection());
    }

    private System.Collections.IEnumerator AdvertiseAfterConnection()
    {
        yield return new WaitUntil(() => ros != null && ros.IsConnected);
        
        if (!hazardAdvertised)
        {
            ros.AdvertiseTopic("/hazard", "std_msgs/String");
            hazardAdvertised = true;
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Hazard"))
        {
            // Prevent multiple alerts for the same hazard
            string hazardId = other.gameObject.name;
            if (detectedHazards.Contains(hazardId)) return;
            
            if (ros != null && ros.IsConnected)
            {
                var hazardMsg = new 
                {
                    message = $"Rover detected hazard: {other.gameObject.name}",
                    type = "collision_alert",
                    hazard_name = other.gameObject.name,
                    position = new 
                    {
                        x = other.transform.position.x,
                        y = other.transform.position.y,
                        z = other.transform.position.z
                    }
                };

                ros.Publish("/hazard", hazardMsg);
                detectedHazards.Add(hazardId);
                
                // ONLY log when actually sent to ROS
                Debug.Log($"⚠️ HAZARD ALERT SENT: {other.gameObject.name} at ({other.transform.position.x:F1}, {other.transform.position.y:F1}, {other.transform.position.z:F1})");
            }
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Hazard"))
        {
            // Allow detecting the same hazard again if rover leaves and re-enters
            detectedHazards.Remove(other.gameObject.name);
        }
    }
}
