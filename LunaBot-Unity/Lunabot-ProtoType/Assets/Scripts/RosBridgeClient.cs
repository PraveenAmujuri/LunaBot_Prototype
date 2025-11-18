using UnityEngine;
using WebSocketSharp;
using Newtonsoft.Json;
using System;
using System.Collections;

public class RosBridgeClient : MonoBehaviour
{
    public string rosbridgeUrl = "ws://172.21.80.1:9090";
    private WebSocket ws;

    public Action<object> OnCmdVelReceived;
    public Action<object> OnCameraReceived;
    public bool IsConnected { get; private set; } = false;

    public float costmapClearDelay = 3.0f;     // /Tuing: increase if ROS services are slow to start

    void Start()
    {
        Application.targetFrameRate = 60;
        QualitySettings.vSyncCount = 0;
        Time.fixedDeltaTime = 0.02f;

        ws = new WebSocket(rosbridgeUrl);

        ws.OnOpen += (s, e) =>
        {
            Debug.Log("ROSBridge connected");
            IsConnected = true;
            StartCoroutine(ClearCostmapsAfterDelay());
        };

        ws.OnMessage += (s, e) =>
        {
            try
            {
                var data = e.Data;
                if (data.Contains("\"/cmd_vel\"") && OnCmdVelReceived != null)
                {
                    OnCmdVelReceived.Invoke(data);
                }
                else if (data.Contains("\"/rover_camera/image_raw\""))
                {
                    if (OnCameraReceived != null) OnCameraReceived.Invoke(data);
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning("Message handling error: " + ex.Message);
            }
        };

        ws.OnError += (s, e) =>
        {
            Debug.LogError("ROSBridge error: " + e.Message);
        };

        ws.OnClose += (s, e) =>
        {
            Debug.Log("ROSBridge closed");
            IsConnected = false;
        };

        ws.ConnectAsync();
    }

    IEnumerator ClearCostmapsAfterDelay()
    {
        yield return new WaitForSeconds(costmapClearDelay);
        if (IsConnected) ClearCostmaps();
    }

    public void ClearCostmaps()
    {
        try
        {
            var serviceCall = new
            {
                op = "call_service",
                service = "/move_base/clear_costmaps",
                args = new { }
            };

            string jsonData = JsonConvert.SerializeObject(serviceCall);
            ws.Send(jsonData);
            Debug.Log("Clear costmaps called");
        }
        catch (Exception e)
        {
            Debug.LogError("Failed to clear costmaps: " + e.Message);
        }
    }

    public void AdvertiseTopic(string topic, string type)
    {
        if (!IsConnected) return;
        try
        {
            var msg = new { op = "advertise", topic = topic, type = type };
            ws.Send(JsonConvert.SerializeObject(msg));
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to advertise {topic}: {e.Message}");
        }
    }

    public void Publish(string topic, object rosMsg)
    {
        if (!IsConnected || ws == null)
        {
            Debug.LogWarning($"Cannot publish to {topic}: Not connected");
            return;
        }
        try
        {
            var msg = new { op = "publish", topic = topic, msg = rosMsg };
            string jsonData = JsonConvert.SerializeObject(msg);
            ws.Send(jsonData);

            // /Tuing: reduce debug frequency for heavy topics; occasional sampling
            if (topic.Contains("camera") && UnityEngine.Random.Range(0, 1000) < 2)
            {
                Debug.Log($"Published camera frame (chars): {jsonData.Length}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to publish to {topic}: {e.Message}");
        }
    }

    public void Subscribe(string topic, string type)
    {
        if (!IsConnected) return;
        try
        {
            var msg = new { op = "subscribe", topic = topic, type = type };
            ws.Send(JsonConvert.SerializeObject(msg));
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to subscribe to {topic}: {e.Message}");
        }
    }

    [ContextMenu("Clear Costmaps Now")]
    public void ClearCostmapsManual()
    {
        if (IsConnected) ClearCostmaps();
        else Debug.LogWarning("Not connected to ROSBridge");
    }

    void OnDestroy()
    {
        try
        {
            if (ws != null) ws.Close();
        }
        catch (Exception e)
        {
            Debug.LogWarning("Error closing WebSocket: " + e.Message);
        }
    }
}
