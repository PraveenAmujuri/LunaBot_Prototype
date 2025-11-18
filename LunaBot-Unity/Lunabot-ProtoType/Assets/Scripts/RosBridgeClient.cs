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
    
    // NEW: Delay before clearing costmaps (seconds)
    public float costmapClearDelay = 3.0f;

    void Start()
    {
            Application.targetFrameRate = 60;
    QualitySettings.vSyncCount = 0;
    Time.fixedDeltaTime = 0.02f;
        ws = new WebSocket(rosbridgeUrl);

        ws.OnOpen += (s, e) =>
        {
            Debug.Log("✅ ROSBridge connected successfully");
            IsConnected = true;
            
            // NEW: Clear costmaps after connection
            StartCoroutine(ClearCostmapsAfterDelay());
        };

        ws.OnMessage += (s, e) =>
        {
            try
            {
                var data = e.Data;
                if (data.Contains("\"/cmd_vel\"") && OnCmdVelReceived != null)
                {
                    Debug.Log("📨 cmd_vel message received");
                    OnCmdVelReceived.Invoke(data);
                }
                else if (data.Contains("\"/rover_camera/image_raw\""))
                {
                    Debug.Log("📷 Camera message received in Unity!");
                    if (OnCameraReceived != null)
                    {
                        OnCameraReceived.Invoke(data);
                    }
                }
            }
            catch (System.Exception ex)
            {
                Debug.LogWarning("⚠️ Message handling error: " + ex.Message);
            }
        };

        ws.OnError += (s, e) =>
        {
            Debug.LogError("❌ ROSBridge error: " + e.Message);
        };

        ws.OnClose += (s, e) =>
        {
            Debug.Log("🔌 ROSBridge connection closed");
            IsConnected = false;
        };

        ws.ConnectAsync();
    }

    // NEW: Coroutine to clear costmaps after scene start
    private IEnumerator ClearCostmapsAfterDelay()
    {
        Debug.Log($"⏳ Waiting {costmapClearDelay}s before clearing costmaps...");
        yield return new WaitForSeconds(costmapClearDelay);
        
        if (IsConnected)
        {
            ClearCostmaps();
        }
    }

    // NEW: Call ROS service to clear costmaps
    public void ClearCostmaps()
    {
        try
        {
            // Call clear_costmaps service
            var serviceCall = new
            {
                op = "call_service",
                service = "/move_base/clear_costmaps",
                args = new { }
            };
            
            string jsonData = JsonConvert.SerializeObject(serviceCall);
            ws.Send(jsonData);
            Debug.Log("🧹 Costmaps cleared! Navigation reset complete.");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"❌ Failed to clear costmaps: {e.Message}");
        }
    }

    public void AdvertiseTopic(string topic, string type)
    {
        if (!IsConnected) return;
        try
        {
            var msg = new { op = "advertise", topic = topic, type = type };
            ws.Send(JsonConvert.SerializeObject(msg));
            Debug.Log($"📢 Advertised topic: {topic}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"❌ Failed to advertise {topic}: {e.Message}");
        }
    }

    public void Publish(string topic, object rosMsg)
    {
        if (!IsConnected || ws == null) 
        {
            Debug.LogWarning($"❌ Cannot publish to {topic}: Not connected");
            return;
        }
        try
        {
            var msg = new { op = "publish", topic = topic, msg = rosMsg };
            string jsonData = JsonConvert.SerializeObject(msg);
            ws.Send(jsonData);
            if (topic.Contains("camera") && UnityEngine.Random.Range(0, 100) < 5)
            {
                Debug.Log($"📤 Published camera frame: {jsonData.Length} chars");
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"❌ Failed to publish to {topic}: {e.Message}");
        }
    }

    public void Subscribe(string topic, string type)
    {
        if (!IsConnected) return;
        try
        {
            var msg = new { op = "subscribe", topic = topic, type = type };
            ws.Send(JsonConvert.SerializeObject(msg));
            Debug.Log($"🔔 Subscribed to: {topic} ({type})");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"❌ Failed to subscribe to {topic}: {e.Message}");
        }
    }

    // NEW: Manual button to clear costmaps (optional)
    [ContextMenu("Clear Costmaps Now")]
    public void ClearCostmapsManual()
    {
        if (IsConnected)
        {
            ClearCostmaps();
        }
        else
        {
            Debug.LogWarning("⚠️ Not connected to ROSBridge. Cannot clear costmaps.");
        }
    }

    void OnDestroy()
    {
        try
        {
            if (ws != null)
            {
                ws.Close();
            }
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"⚠️ Error closing WebSocket: {e.Message}");  // FIXED: Capital M
        }
    }
}
