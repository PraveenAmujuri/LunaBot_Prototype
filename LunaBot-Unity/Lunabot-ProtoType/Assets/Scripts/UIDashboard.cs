using UnityEngine;
using UnityEngine.UI;

public class UIDashboard : MonoBehaviour {
    public Text tempText, oxygenText, pressureText, alertText;

    public void UpdateEnv(float temp, float oxy, float pressure){
        tempText.text = string.Format("Temp: {0:0.0} °C", temp);
        oxygenText.text = string.Format("O₂: {0:0.0}%", oxy);
        pressureText.text = string.Format("Pressure: {0:0.00} kPa", pressure);
    }

    public void ShowAlert(string a){ alertText.text = a; alertText.color = Color.red; }
}