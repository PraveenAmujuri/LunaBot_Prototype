using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class LidarSensor : MonoBehaviour
{
    public RosBridgeClient ros;
    
    [Header("LIDAR Configuration")]
    public int numRays = 180;
    public float fov = 360.0f;
    public float maxRange = 20.0f;
    public float scanRate = 10f;

    [Header("Layer Settings")]
    public LayerMask obstacleMask;

    [Header("LaserScan ROS Settings")]
    public string scanTopic = "/scan";
    public string frameId = "lidar_link";

    [Header("Debug")]
    public bool showDebugRays = true;  // ← ENABLED!
    public Color hitColor = Color.red;
    public Color missColor = Color.green;

    void Start()
    {
        if (ros == null)
        {
            Debug.LogError("❌ LidarSensor: ROS Bridge not assigned!");
            return;
        }
        
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // CRITICAL: Check rover's forward direction!
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        Debug.Log("═══════════════════════════════════════════════");
        Debug.Log("🔍 LIDAR Sensor Initialized");
        Debug.Log($"   Rays: {numRays} | FOV: {fov}° | Range: {maxRange}m");
        Debug.Log($"   Position: {transform.position}");
        Debug.Log($"   Forward: {transform.forward}");  // ← CHECK THIS!
        Debug.Log($"   Rotation: {transform.rotation.eulerAngles}");
        Debug.Log($"   Scanning: {LayerMaskToString(obstacleMask)}");
        Debug.Log("═══════════════════════════════════════════════");
        
        StartCoroutine(PublishLaserScan());
    }

    IEnumerator PublishLaserScan()
    {
        float interval = 1f / scanRate;
        
        while (true)
        {
            yield return new WaitForSeconds(interval);

            float angleMin = Mathf.Deg2Rad * -fov / 2f;
            float angleMax = Mathf.Deg2Rad * fov / 2f;
            float angleIncrement = (angleMax - angleMin) / (numRays - 1);

            List<float> ranges = new List<float>(numRays);
            int obstacleCount = 0;
            float minObstacleDist = maxRange;

            // Perform raycasts
            for (int i = 0; i < numRays; i++)
            {
                float angle = angleMin + i * angleIncrement;
                
                // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
                // FIXED: Use rover's forward, not LIDAR's transform.forward!
                // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
                Vector3 baseForward = transform.parent != null ? transform.parent.forward : transform.forward;
                Vector3 dir = Quaternion.Euler(0, Mathf.Rad2Deg * angle, 0) * baseForward;
                // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

                RaycastHit hit;
                bool didHit = Physics.Raycast(transform.position, dir, out hit, maxRange, obstacleMask);

                if (didHit)
                {
                    ranges.Add(hit.distance);
                    obstacleCount++;
                    
                    if (hit.distance < minObstacleDist)
                        minObstacleDist = hit.distance;
                    
                    if (showDebugRays && i % 5 == 0)
                    {
                        Debug.DrawRay(transform.position, dir * hit.distance, hitColor, interval);
                    }
                }
                else
                {
                    ranges.Add(maxRange);
                    
                    if (showDebugRays && i % 10 == 0)
                    {
                        Debug.DrawRay(transform.position, dir * maxRange, missColor, interval);
                    }
                }
            }

            // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
            // CRITICAL DEBUG: Show minimum distance!
            // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
            if (obstacleCount > 0)
            {
                Debug.Log($"📡 LIDAR: {obstacleCount}/{numRays} rays hit | MIN DISTANCE: {minObstacleDist:F1}m");
            }
            // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

            // Publish
            var scanMsg = new
            {
                header = new 
                {
                    stamp = new 
                    { 
                        secs = (int)Time.time, 
                        nsecs = (int)((Time.time % 1) * 1e9) 
                    },
                    frame_id = frameId
                },
                angle_min = angleMin,
                angle_max = angleMax,
                angle_increment = angleIncrement,
                time_increment = 0.0f,
                scan_time = interval,
                range_min = 0.08f,
                range_max = maxRange,
                ranges = ranges.ToArray(),
                intensities = new float[numRays]
            };

            ros.Publish(scanTopic, scanMsg);
        }
    }

    string LayerMaskToString(LayerMask mask)
    {
        string layers = "";
        for (int i = 0; i < 32; i++)
        {
            if ((mask.value & (1 << i)) != 0)
            {
                string layerName = LayerMask.LayerToName(i);
                if (!string.IsNullOrEmpty(layerName))
                {
                    layers += layerName + " ";
                }
            }
        }
        return string.IsNullOrEmpty(layers) ? "NONE" : layers;
    }

    void OnDrawGizmos()
    {
        if (!showDebugRays || !Application.isPlaying) return;

        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(transform.position, 0.2f);
        
        // Draw forward direction
        Gizmos.color = Color.cyan;
        Vector3 fwd = transform.parent != null ? transform.parent.forward : transform.forward;
        Gizmos.DrawRay(transform.position, fwd * 2f);
    }
}
