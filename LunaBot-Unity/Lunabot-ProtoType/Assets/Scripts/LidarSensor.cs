using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class LidarSensor : MonoBehaviour
{
    public RosBridgeClient ros;

    [Header("LIDAR Configuration")]
    public int numRays = 180;            // Tuning: lower = faster, higher = finer scan
    public float fov = 360.0f;          // Tuning: set to sensor FOV
    public float maxRange = 20.0f;      // Tuning: set to actual lidar max range
    public float scanRate = 10f;        // Tuning: reduce if CPU/bandwidth constrained

    [Header("Layer Settings")]
    public LayerMask obstacleMask;

    [Header("LaserScan ROS Settings")]
    public string scanTopic = "/scan";
    public string frameId = "lidar_link";

    [Header("Debug")]
    public bool showDebugRays = true;
    public Color hitColor = Color.red;
    public Color missColor = Color.green;

    void Start()
    {
        if (ros == null)
        {
            Debug.LogError("LidarSensor: ROS Bridge not assigned.");
            return;
        }

        StartCoroutine(PublishLaserScan());
    }

    private double GetROSTime()
    {
        return (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
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

            for (int i = 0; i < numRays; i++)
            {
                float angle = angleMin + i * angleIncrement;

                Vector3 baseForward = transform.parent != null ? transform.parent.forward : transform.forward;
                Vector3 dir = Quaternion.Euler(0, Mathf.Rad2Deg * angle, 0) * baseForward;

                RaycastHit hit;
                bool didHit = Physics.Raycast(transform.position, dir, out hit, maxRange, obstacleMask);

                if (didHit)
                {
                    ranges.Add(hit.distance);
                    obstacleCount++;

                    if (hit.distance < minObstacleDist)
                        minObstacleDist = hit.distance;

                    if (showDebugRays && i % 5 == 0)
                        Debug.DrawRay(transform.position, dir * hit.distance, hitColor, interval);
                }
                else
                {
                    ranges.Add(maxRange);

                    if (showDebugRays && i % 10 == 0)
                        Debug.DrawRay(transform.position, dir * maxRange, missColor, interval);
                }
            }

            if (obstacleCount > 0)
                Debug.Log($"LIDAR hits: {obstacleCount}/{numRays}  min:{minObstacleDist:F1}m");

            double rosTime = GetROSTime();
            var scanMsg = new
            {
                header = new
                {
                    stamp = new
                    {
                        secs = (int)rosTime,
                        nsecs = (int)((rosTime % 1) * 1e9)
                    },
                    frame_id = frameId
                },
                angle_min = angleMin,
                angle_max = angleMax,
                angle_increment = angleIncrement,
                time_increment = 0.0f,
                scan_time = interval,
                range_min = 0.08f,        // Tuning: set to sensor minimum reliable range
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
                    layers += layerName + " ";
            }
        }
        return string.IsNullOrEmpty(layers) ? "NONE" : layers;
    }

    void OnDrawGizmos()
    {
        if (!showDebugRays || !Application.isPlaying) return;

        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(transform.position, 0.2f);

        Gizmos.color = Color.cyan;
        Vector3 fwd = transform.parent != null ? transform.parent.forward : transform.forward;
        Gizmos.DrawRay(transform.position, fwd * 2f);
    }
}
