using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PenaltyCollidersBox : MonoBehaviour
{
    public RobotControllerAgent parentAgent;

    private void OnTriggerEnter(Collider other)
    {
        if (parentAgent != null)
        {
            if (other.transform.CompareTag("RobotInternal"))
            {
               parentAgent.ObstacleHitPenalty();
            }
            else
            {
               parentAgent.BoxNotHitReward();
            }
        }
        
    }
}
