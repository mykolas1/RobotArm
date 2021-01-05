using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ComponentCollision : MonoBehaviour
{

    public RobotControllerAgent parentAgent;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    void OnCollisionEnter (Collision col)
    {
        if (col.gameObject.name == "Obstacle1" ||
            col.gameObject.name == "Obstacle2" ||
            col.gameObject.name == "Obstacle3" ||
            col.gameObject.name == "Obstacle4") {
            Debug.Log("Collision");
            parentAgent.UpdateNearestComponent();
        }
    }
}
