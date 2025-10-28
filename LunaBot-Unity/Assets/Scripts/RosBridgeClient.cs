using UnityEngine;
using WebSocketSharp;
using System;
using System.Text;
using Newtonsoft.Json;

public class RosBridgeClient : MonoBehaviour {
    public string rosbridgeUrl = "ws://localhost:9090";
    private WebSocket ws;

    void Start(){
        ws = new WebSocket(rosbridgeUrl);
        ws.OnOpen += (s,e)=> Debug.Log("ROSBridge connected");
        ws.OnMessage += (s,e)=> { };
        ws.OnError += (s,e)=> Debug.LogError("ROSBridge error: " + e.Message);
        ws.OnClose += (s,e)=> Debug.Log("ROSBridge closed");
        ws.ConnectAsync();
    }

    public void AdvertiseTopic(string topic, string type){
        var msg = new { op="advertise", topic=topic, type=type };
        ws.Send(JsonConvert.SerializeObject(msg));
    }

    public void Publish(string topic, object rosMsg){
        var msg = new { op="publish", topic=topic, msg=rosMsg };
        ws.Send(JsonConvert.SerializeObject(msg));
    }

    public void Subscribe(string topic){
        var msg = new { op="subscribe", topic=topic };
        ws.Send(JsonConvert.SerializeObject(msg));
    }

    void OnDestroy(){ if(ws!=null) ws.Close(); }
}