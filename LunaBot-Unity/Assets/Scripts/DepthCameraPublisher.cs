using UnityEngine;
using System.Collections;
using System;

public class DepthCameraPublisher : MonoBehaviour {
    public Camera cam;
    public RenderTexture rt;
    public RosBridgeClient ros;
    public int publishHz = 5;

    IEnumerator Start(){
        while(true){
            Texture2D tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
            RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0,0,rt.width, rt.height),0,0);
            tex.Apply();
            byte[] jpg = tex.EncodeToJPG();
            var msg = new { header = new { stamp = new { secs = (int)Time.time } }, height = tex.height, width = tex.width, encoding = "jpeg", is_bigendian = 0, step = jpg.Length, data = Convert.ToBase64String(jpg) };
            ros.Publish("/camera/image_raw", msg);
            UnityEngine.Object.Destroy(tex);
            yield return new WaitForSeconds(1f/publishHz);
        }
    }
}