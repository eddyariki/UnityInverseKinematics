using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKController : MonoBehaviour
{
    public GameObject IKPrefab;
    public GameObject follow;
    public GameObject testTarget;
    public GameObject testBaseTarget;
    public int ikCount = 10;
    public float outerRadius = 20f;
    public float innerRadius = 10f;
    [Range(0,1f)]
    public float noise = 0.1f;
    public float speed = 1f;
    public float height = 1f;
    public float area = 2f;

    private GameObject[] IKPrefabs;
    private Vector3[] targets;
    private Vector3[] baseTargets;
    private float[] targetsTheta;
    private float[] targetsPhi;
    private bool[] isStepping; 
    public Vector3[] pos { get { return baseTargets; } }
    void Start()
    {
        IKPrefabs = new GameObject[ikCount];
        targets = new Vector3[ikCount];
        baseTargets = new Vector3[ikCount];
        targetsTheta = new float[ikCount];
        targetsPhi = new float[ikCount];
        isStepping = new bool[ikCount];
        for(int i = 0; i<IKPrefabs.Length; i++)
        {
            IKPrefabs[i] = Instantiate(IKPrefab, transform);
            IKPrefabs[i].name = "IK" + i;
            
        }
        //InitWalkingIK();
    }

    // Update is called once per frame
    void Update()
    {
        //follow.transform.position+= Vector3.left * speed * Time.deltaTime;
        for (int i = 0; i < IKPrefabs.Length; i++)
        {
            InverseKinematics ik = IKPrefabs[i].GetComponent<InverseKinematics>();
            //SetTargetsOnSphere(i);  //Set the targets
            //SetBaseTargetsOnSphere(i);
            //WalkingIK(i, ik.ArmLengthSquared);
            //ik.setBase(baseTargets[i]);
            //ik.setTarget(targets[i]);
            ik.setBase(testBaseTarget.transform.position);
            ik.setTarget(testTarget.transform.position);
            ik.UpdateIK();
        }
    }
    void SetTargetsOnSphere(int index)
    {
        float theta = Random.Range(0, Mathf.PI);
        float phi = Random.Range(0, Mathf.PI * 2);
        targetsTheta[index] = theta;
        targetsPhi[index] = phi;
        targets[index] = new Vector3(outerRadius * Mathf.Sin(theta) * Mathf.Cos(phi), outerRadius * Mathf.Sin(theta) * Mathf.Sin(phi), outerRadius * Mathf.Cos(theta));
        
    }
    void SetBaseTargetsOnSphere(int index)
    {
        float theta = Random.Range(0, Mathf.PI);
        float phi = Random.Range(0, Mathf.PI * 2);
        targetsTheta[index] = theta;
        targetsPhi[index] = phi;
        baseTargets[index] = new Vector3(innerRadius * Mathf.Sin(theta) * Mathf.Cos(phi), innerRadius * Mathf.Sin(theta) * Mathf.Sin(phi), innerRadius * Mathf.Cos(theta));
    }
    void InitWalkingIK()
    {
        for(int i=0; i<ikCount; i++)
        {
            float x = Random.Range(-area, area);
            float z = Random.Range(-area, area);
            baseTargets[i] = new Vector3(x*0.1f, 0, z ) + transform.position;
            targets[i] = new Vector3(x*8,-height,z);
            isStepping[i] = true;
        }
    }
    void WalkingIK(int index, float armLengthSquared)
    {
        float x = Random.Range(-area, area);
        float z = Random.Range(-area, area);
        if (isStepping[index] == false)
        {
            
            Vector3 dest = baseTargets[index] + new Vector3(-height * 0.2f, -height * 0.1f, z*2);
            targets[index] += (-targets[index] + dest) * speed * Time.deltaTime;
            if (Vector3.Distance(targets[index] ,dest)<height*0.1f)
            {
                isStepping[index] = true;
                //Debug.Log("Became True!");
            }
        }else if(isStepping[index] == true && targets[index].y >= -height)
        {
            targets[index] += new Vector3(-height*0.2f, -height*0.2f, 0) *speed * Time.deltaTime;
        }


        baseTargets[index] += Vector3.left *speed*Time.deltaTime + new Vector3(x*Time.deltaTime, -baseTargets[index].y * 0.3f + x * 0.5f, -baseTargets[index].z*0.3f+ z *0.7f); 
        
        float dist = Vector3.SqrMagnitude(targets[index] - baseTargets[index]);
        if(dist > armLengthSquared && targets[index].y < -height) 
        { 
            isStepping[index] = false;
        }
       
    }
}
