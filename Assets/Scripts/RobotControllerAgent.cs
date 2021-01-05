using System;
using System.Data.Common;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;
using UnityEngine;
using System.Collections;

public class RobotControllerAgent : Agent
{
   [SerializeField]
   private GameObject[] armAxes;
   [SerializeField] 
   private GameObject endEffector;

   public bool trainingMode;
   private bool inFrontOfComponent = false;
   public GameObject nearestComponent;
   public GameObject obstacle1;
   public GameObject obstacle2;
   public GameObject obstacle3;
   public GameObject obstacle4;
   public GameObject arm;
   private Ray rayToTest = new Ray();
   private float[] angles = new float[8];
   private KinematicsCalculator calculator;
   private float beginDistance;
   private float prevBest;
   bool  inrange2 = false, inrange3 = false, inrange4 = false, inrange5 = false;
   private float baseAngle;
   private const float stepPenalty = -0.0001f;

   private float bestTime;
   private float startTime;
   private float absoluteSumOfAngles;
   private float pastAbsoluteSumOfAngles = 10000;
    private void Start()
   {
        obstacle1 = GameObject.Find("Obstacle1");
        obstacle2 = GameObject.Find("Obstacle2");
        obstacle3 = GameObject.Find("Obstacle3");
        obstacle4 = GameObject.Find("Obstacle4");
    }

   public override void Initialize()
   {
      ResetAllAxis();
      MoveToSafeRandomPosition();
      if (!trainingMode) MaxStep = 0;
   }

   private void ResetAllAxis()
   {
      armAxes.All(c =>
      {
         c.transform.localRotation =  Quaternion.Euler(0f, 0f, 0f);
         return true;
      });
   }

   public override void OnEpisodeBegin()
   {
      if(trainingMode)
         ResetAllAxis();
      startTime = Time.time;
      MoveToSafeRandomPosition();
      UpdateNearestComponent();
   }

   public void UpdateNearestComponent()
   {
      if (trainingMode)
      {
         inFrontOfComponent = UnityEngine.Random.value > 0.5f;
      }
      if(!inFrontOfComponent)
        {
            nearestComponent.transform.position = transform.position + new Vector3(Random.Range(0.5f, 0.8f), Random.Range(0.3f, 0.4f), Random.Range(0.5f, 0.8f));
        }
        else
        {
            nearestComponent.transform.position = endEffector.transform.TransformPoint(Vector3.zero) + new Vector3(Random.Range(0.05f,0.18f),Random.Range(0.05f,0.18f), Random.Range(0.05f,0.18f));
        }
      beginDistance = Vector3.Distance(endEffector.transform.TransformPoint(Vector3.zero), nearestComponent.transform.position);
      prevBest = beginDistance;
      
      baseAngle = Mathf.Atan2( transform.position.x - nearestComponent.transform.position.x, transform.position.z - nearestComponent.transform.position.z) * Mathf.Rad2Deg;
      if (baseAngle < 0) baseAngle = baseAngle + 360f;
   }

   /// <summary>
   /// Markov Decision Process - Observes state for the current time step
   /// </summary>
   /// <param name="sensor"></param>
   public override void CollectObservations(VectorSensor sensor)
   {
        sensor.AddObservation(angles);
      sensor.AddObservation(obstacle1.transform.position.normalized);
      sensor.AddObservation(obstacle2.transform.position.normalized);
      sensor.AddObservation(obstacle3.transform.position.normalized);
      sensor.AddObservation(obstacle4.transform.position.normalized);
      sensor.AddObservation(transform.position.normalized);
      sensor.AddObservation(nearestComponent.transform.position.normalized);
      //sensor.AddObservation(nearestObstacle.transform.position.normalized);
      sensor.AddObservation(endEffector.transform.TransformPoint(Vector3.zero).normalized);
      Vector3 toComponent = (nearestComponent.transform.position - endEffector.transform.TransformPoint(Vector3.zero));
      sensor.AddObservation(toComponent.normalized);
      sensor.AddObservation(Vector3.Distance(nearestComponent.transform.position,endEffector.transform.TransformPoint(Vector3.zero)));
      sensor.AddObservation(StepCount / 5000);
   }

   public override void OnActionReceived(float[] vectorAction)
   {
      angles = vectorAction;
      if (trainingMode)
      {
         // Translate the floating point actions into Degrees of rotation for each axis
         armAxes[0].transform.localRotation =
            Quaternion.AngleAxis(angles[0] * 180f, armAxes[0].GetComponent<Axis>().rotationAxis);
         armAxes[1].transform.localRotation =
            Quaternion.AngleAxis(angles[1] * 90f, armAxes[1].GetComponent<Axis>().rotationAxis);
         armAxes[2].transform.localRotation =
            Quaternion.AngleAxis(angles[2] * 80f, armAxes[2].GetComponent<Axis>().rotationAxis);
         armAxes[3].transform.localRotation =
            Quaternion.AngleAxis(angles[3] * 80f, armAxes[3].GetComponent<Axis>().rotationAxis);
         armAxes[4].transform.localRotation =
            Quaternion.AngleAxis(angles[4] * 80f, armAxes[4].GetComponent<Axis>().rotationAxis);
         armAxes[5].transform.localRotation =
            Quaternion.AngleAxis(angles[5] * 80f, armAxes[5].GetComponent<Axis>().rotationAxis);
         armAxes[6].transform.localRotation =
            Quaternion.AngleAxis(angles[6] * 80f, armAxes[6].GetComponent<Axis>().rotationAxis);
         armAxes[7].transform.localRotation =
            Quaternion.AngleAxis(angles[7] * 90f, armAxes[7].GetComponent<Axis>().rotationAxis);


            float distance = Vector3.Distance(endEffector.transform.TransformPoint(Vector3.zero),
            nearestComponent.transform.position);
         float diff = beginDistance - distance;
         
         if (distance > prevBest) // tiesiog absoliuti distancija. TODO
         {
                // Penalty if the arm moves away from the closest position to target
                //Debug.LogWarning("PENALTY DISTANCE " + (prevBest - distance));
                float a = distance * (-1);
                AddReward(a);
                //AddReward(distance * (-1)); - neapsimoko
         }
         else
         {
                // Reward if the arm moves closer to target
                //Debug.LogWarning("REWARD DISTANCE " + diff);

                AddReward(diff);
            prevBest = distance;
         }
         AddReward(stepPenalty);
      }
   }

   public void GroundHitPenalty()
   {
      AddReward(-1f);
      EndEpisode();
   }

    public void BoxNotHitReward()
    {
        AddReward(0.5f);
    }

    public void ObstacleHitPenalty()
    {
        AddReward(-0.0005f);
        EndEpisode();
    }

    private void OnTriggerEnter(Collider other)
   {
      JackpotReward(other);
   }

   public void JackpotReward(Collider other)
   {
      if (other.transform.CompareTag("Components"))
      {
         float SuccessReward = 1.0f;
         float bonus = Mathf.Clamp01(Vector3.Dot(nearestComponent.transform.up.normalized,
                          endEffector.transform.up.normalized));
         float reward = SuccessReward + bonus;
         absoluteSumOfAngles = 0;

         absoluteSumOfAngles = absoluteSumOfAngles + armAxes[2].transform.localEulerAngles.z;
         absoluteSumOfAngles = absoluteSumOfAngles + armAxes[3].transform.localEulerAngles.z;
         absoluteSumOfAngles = absoluteSumOfAngles + armAxes[4].transform.localEulerAngles.z;
         absoluteSumOfAngles = absoluteSumOfAngles + armAxes[5].transform.localEulerAngles.z;
         absoluteSumOfAngles = absoluteSumOfAngles + armAxes[6].transform.localEulerAngles.z;
         absoluteSumOfAngles = absoluteSumOfAngles + armAxes[7].transform.localEulerAngles.z;

         AddBonusOrPenaltyOnAngleAbsSize(Math.Abs(absoluteSumOfAngles));

         AddTimeReward(reward);

         if (float.IsInfinity(reward) || float.IsNaN(reward)) return;
          Debug.LogWarning("Great! Component reached. Positive reward:" + reward );
         AddReward(reward);
         //EndEpisode();
         UpdateNearestComponent();
      }
   }  

    private void AddTimeReward(float reward)
    {
        //time 
        float timeSpent = Time.time - startTime;

        if (timeSpent < bestTime)
        {
            Debug.LogWarning("BETTER TIME");
            reward = reward + 0.5f;
        }
        else
        {
            reward = reward - 0.5f;
        }
    }

    private void AddBonusOrPenaltyOnAngleAbsSize(float absolute)
    {
        if (0.0f < absolute && absolute < 100.0f)
        {
            AddReward(0.5f);
        }
        else if (100.0f <= absolute && absolute < 200.0f)
        {
            AddReward(0.4f);
        }
        else if (200.0f <= absolute && absolute < 300.0f)
        {
            AddReward(0.3f);
        }
        else if (300.0f <= absolute && absolute < 400.0f)
        {
            AddReward(0.2f);
        }
        else if (400.0f <= absolute && absolute < 500.0f)
        {
            AddReward(0.1f);
        }
        else if (500.0f <= absolute && absolute < 600.0f)
        {
            AddReward(0.0f);
        }
        else if (600.0f <= absolute && absolute < 700.0f)
        {
            AddReward(-0.1f);
        }
        else if (700.0f <= absolute && absolute < 800.0f)
        {
            AddReward(-0.2f);
        }
        else if (800.0f <= absolute && absolute < 900.0f)
        {
            AddReward(-0.3f);
        }
        else if (900.0f <= absolute && absolute < 1000.0f)
        {
            AddReward(-0.4f);
        }
        else
        {
            AddReward(-0.5f);
        }
    }

   // private float[] NormalizedAngles()
   // {
   //    float[] normalized = new float[6];
   //    for (int i = 0; i < 6; i++)
   //    {
   //       normalized[i] = angles[i] / 360f;
   //    }
   //
   //    return normalized;
   // }

   private void MoveToSafeRandomPosition()
   {
      int maxTries = 100;
      
      while (maxTries > 0)
      {
         armAxes.All(axis =>
            {
               Axis ax = axis.GetComponent<Axis>();
               Vector3 angle = ax.rotationAxis * Random.Range(ax.MinAngle, ax.MaxAngle);
               ax.transform.localRotation = Quaternion.Euler(angle.x, angle.y, angle.z);
               return true;
            }
         );
         Vector3 tipPosition = endEffector.transform.TransformPoint(Vector3.zero);
         Plane groundPlane = new Plane(Vector3.up, Vector3.zero);
         float distanceFromGround = groundPlane.GetDistanceToPoint(tipPosition);
         if (distanceFromGround > 0.1f && distanceFromGround <= 1f && tipPosition.y > 0.01f)
         {
            break;
         }
         maxTries--;
      }
   }
   private void Update()
   {
      if(nearestComponent != null)
         Debug.DrawLine(endEffector.transform.position,nearestComponent.transform.position, Color.green);
   }
}
