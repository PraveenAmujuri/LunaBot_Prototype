using UnityEngine;

public class FPSDisplay : MonoBehaviour
{
    float deltaTime = 0.0f;

    void Update()
    {
        deltaTime += (Time.unscaledDeltaTime - deltaTime) * 0.1f;
    }

    void OnGUI()
    {
        int w = Screen.width, h = Screen.height;

        GUIStyle style = new GUIStyle();
        Rect rect = new Rect(10, 10, w, h * 2 / 100);
        style.alignment = TextAnchor.UpperRight;
        style.fontSize = h * 2 / 100; 
        
        float fps = 1.0f / deltaTime;
        
        // Color code the FPS
        if (fps < 10)
            style.normal.textColor = Color.red;
        else if (fps < 30)
            style.normal.textColor = Color.yellow;
        else
            style.normal.textColor = Color.green;
        
        string text = string.Format("{0:0.} FPS\nFrame: {1:0.}ms", fps, deltaTime * 1000.0f);
        GUI.Label(rect, text, style);
    }
}
