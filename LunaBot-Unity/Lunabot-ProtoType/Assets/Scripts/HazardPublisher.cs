using UnityEngine;

public class HazardPublisher : MonoBehaviour
{
    public RosBridgeClient ros;

    void Start()
    {
        StartCoroutine(WaitForConnectionAndPublish());
    }

    private System.Collections.IEnumerator WaitForConnectionAndPublish()
    {
        yield return new WaitUntil(() => ros != null && ros.IsConnected);

        ros.AdvertiseTopic("/hazard", "std_msgs/String");
        yield return new WaitForSeconds(1f);

        GameObject[] hazards = GameObject.FindGameObjectsWithTag("Hazard");

        foreach (GameObject hazard in hazards)
        {
            Vector3 pos = hazard.transform.position;
            
            var hazardMsg = new 
            {
                message = $"Static hazard location: {hazard.name}",
                type = "static_hazard",
                hazard_name = hazard.name,
                position = new 
                {
                    x = pos.x,
                    y = pos.y,
                    z = pos.z
                }
            };

            ros.Publish("/hazard", hazardMsg);
            yield return new WaitForSeconds(0.1f);
        }
        
        // ONLY log the summary
        Debug.Log($"📡 Published {hazards.Length} static hazard locations to ROS");
    }
}
