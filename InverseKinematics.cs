using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class InverseKinematics : MonoBehaviour
{
    //Mesh Generator Variables
    public int ringPoints;
    public bool generateMesh;
    public int tangentW = 1;
    public float radius = 0.5f;
    private Mesh mesh;
    private Vector3[] normals;
    private Vector4[] tangents;
    private Vector3[] meshVerts;
    private Vector2[] uvs;
    private int[] triangles;
    private float angles;




    //IK Variables 
    public Vector3 target; //Target for tip 
    public Vector3 baseTarget; //Target for base 
    public int segments = 1;
    public float segmentLength = 5f;
    public bool calculateForward = false;
    public bool showGizmos = false;
    public bool useLocal = true;
    private OrientedPoint[] arm;
    private int joints;
    private float armLength;
    private float armLengthSquared;
    public float ArmLengthSquared { get { return armLengthSquared; } }
    private void OnDrawGizmos()
    {
        if (arm != null && showGizmos)
        {
            for (int i = 0; i < arm.Length; i++)
            {
                if (i != arm.Length - 1)
                {
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawLine(arm[i].position, arm[i + 1].position);
                }
                Gizmos.color = Color.red;
                //Gizmos.DrawWireSphere(arm[i].position, 1);
                Gizmos.color = Color.blue;
                for (int j = 0; j < 10; j++)
                {
                    Gizmos.DrawLine(arm[i].position, arm[i].LocalToWorld(new Vector3(5 * Mathf.Cos(j * Mathf.PI * 2 / 10), 5 * Mathf.Sin(j * Mathf.PI * 2 / 10), 0)));
                }
                Gizmos.color = Color.green;
                Gizmos.DrawLine(arm[i].position, arm[i].LocalToWorld(Vector3.up));
                Gizmos.DrawLine(arm[i].position, arm[i].LocalToWorld(Vector3.right));
                Gizmos.DrawLine(arm[i].position, arm[i].LocalToWorld(Vector3.forward));

            }
        }
    }

    private void Awake()
    {
        //IK Setup
        joints = segments + 1;
        arm = new OrientedPoint[joints];
        armLength = 0f;
        for (int i = 0; i < joints; i++)
        {
            arm[i] = new OrientedPoint(Vector3.forward * segmentLength * i, Quaternion.identity, segmentLength);
            armLength += segmentLength;
        }
        armLengthSquared = armLength * armLength;
        //Mesh Generation Setup
        mesh = new Mesh();
        mesh.name = "IK";
        GetComponent<MeshFilter>().mesh = this.mesh;
        triangles = new int[ringPoints * (segments - 2) * 6 + 2 * 3 * ringPoints + 3]; //Check
        meshVerts = new Vector3[ringPoints * (joints - 2) + 2];
        normals = new Vector3[ringPoints * (joints - 2) + 2];
        tangents = new Vector4[ringPoints * (joints - 2) + 2]; 
        uvs = new Vector2[ringPoints * (joints - 2) + 2];
        angles = (Mathf.PI * 2) / ringPoints;
        if (generateMesh)
        {
            GenerateMesh();
        }
    }
  

    public void UpdateIK()
    {
        CalculateIK();
        if (calculateForward)
        {
            CalculateFK();
        }
        GenerateMesh();

    }
    public void setTarget(Vector3 t)
    {
        target = t;
    }
    public void setBase(Vector3 b)
    {
        baseTarget = b;
    }
    private void CalculateIK()
    {
        //Calculates the positions inversely; AKA bringing the arm back to the base 
        Vector3 up = Vector3.up;
        for (int i = joints - 1; i >= 0; i--)
        {
            if (useLocal) up = arm[i].LocalToWorld(Vector3.up);
            if (i == joints - 1)
            {
                arm[i].position = target- Vector3.Normalize(target - arm[i].position) *arm[i].length / 10f;
                arm[i].rotation = Quaternion.LookRotation(target - arm[i].position, up);
            }
            else
            {
                arm[i].position = arm[i + 1].position - Vector3.Normalize(arm[i + 1].position - arm[i].position) * arm[i].length;
                arm[i].rotation = Quaternion.LookRotation(arm[i + 1].position - arm[i].position, up);
            }
        }
    }
    private void CalculateFK()
    {
        //Calculates the positions forward; AKA bringing the arm back to the base 
        Vector3 up = Vector3.up;
        for (int i = 0; i < joints; i++)
        {
            if (useLocal) up = arm[i].LocalToWorld(Vector3.up);
            if (i == 0)
            {
                arm[i].position = baseTarget;
                arm[i].rotation = Quaternion.LookRotation(arm[i + 1].position - arm[i].position, up);
            }
            else
            {
                arm[i].position = arm[i - 1].position - Vector3.Normalize(arm[i - 1].position - arm[i].position) * arm[i].length;
                arm[i].rotation = Quaternion.LookRotation(arm[i - 1].position - arm[i].position, up);
            }

        }
    }

    private void GenerateMesh()
    {
        GenerateVerts();
        GenerateTris();
        mesh.Clear();
        mesh.vertices = meshVerts;
        mesh.triangles = triangles;
        mesh.normals = normals;
        mesh.tangents = tangents;
        mesh.uv = uvs;
    }
    private void GenerateVerts()
    {
        //Generates the verts from the IK
        for (int j = 0, s = 0; j < joints; j++)
        {
            //s is used for vertice index
            //j is used for joint index 
            if (j == 0 || j == joints - 1)
            {
                meshVerts[s] = arm[j].position;
                uvs[s] = new Vector2(0.5f,0.5f);
                normals[s] = arm[j].rotation.eulerAngles.normalized;
                Vector3 tangent3 = Vector3.Cross(normals[s], arm[j].LocalToWorldDirection(Vector3.forward));
                tangents[s] = new Vector4(tangent3.x, tangent3.y, tangent3.z, tangentW);
                s++;
            }
            else
            {
                for (int i = 0; i < ringPoints; i++)
                {
                    //Create a ring around the oriented point
                    Vector3 ring = new Vector3(radius * Mathf.Cos(angles * i), radius * Mathf.Sin(angles * i), 0);
                    Vector3 ringNormal = new Vector3(Mathf.Cos(angles * i), Mathf.Sin(angles * i), 0); //No need for normalize()
                    meshVerts[s] = arm[j].LocalToWorld(ring);
                    uvs[s] = new Vector2((angles * i) / (Mathf.PI * 2), arm[j].length * j / armLength);
                    normals[s] = arm[j].LocalToWorldDirection(ring);
                    Vector3 tangent3 = Vector3.Cross(ringNormal, Vector3.forward);
                    tangents[s] = arm[j].LocalToWorldDirection(new Vector4(tangent3.x, tangent3.y, tangent3.z, tangentW)); //Check 
                    s++;
                }

            }
        }
    }

    private void GenerateTris()
    {
        int vert = 0;
        int tris = 0;

        for (int y = 0; y < segments; y++)
        {
            if (y == 0)
            {
                //The bottom
                for (int x = 1; x < ringPoints; x++)
                {
                    triangles[tris + 0] = 0;
                    triangles[tris + 1] = x;
                    triangles[tris + 2] = x + 1;
                    tris += 3;
                }
                triangles[tris] = 0;
                triangles[tris + 1] = ringPoints;
                triangles[tris + 2] = 1;
                tris += 3;
                vert++;
            }
            else if (y == segments - 1)
            {
                //The tip
                for (int x = 0; x < ringPoints + 1; x++)
                {
                    triangles[tris + 0] = meshVerts.Length - 1;
                    triangles[tris + 1] = vert;
                    if (x == 0)
                    {
                        triangles[tris + 2] = meshVerts.Length - 2;
                    }
                    else
                    {
                        triangles[tris + 2] = vert - 1;
                    }

                    tris += 3;
                    vert++;
                }

            }
            else
            {
                for (int x = 0; x < ringPoints; x++)
                {
                    if (vert % ringPoints == 0)
                    {
                        triangles[tris + 0] = vert;
                        triangles[tris + 1] = vert + ringPoints;
                        triangles[tris + 2] = vert - ringPoints + 1;

                        triangles[tris + 3] = vert - ringPoints + 1;
                        triangles[tris + 4] = vert + ringPoints;
                        triangles[tris + 5] = vert + 1;
                    }
                    else
                    {
                        triangles[tris + 0] = vert;
                        triangles[tris + 1] = vert + ringPoints;
                        triangles[tris + 2] = vert + 1;

                        triangles[tris + 3] = vert + 1;
                        triangles[tris + 4] = vert + ringPoints;
                        triangles[tris + 5] = vert + ringPoints + 1;
                    }
                    vert++;
                    tris += 6;
                }
            }
        }
    }
}





public struct OrientedPoint
{
    //https://www.youtube.com/watch?v=o9RK6O2kOKo&t=2471s
    //From: "Unite 2015 - A coder's guide to spline-based procedural geometry"
    public Vector3 position;
    public Quaternion rotation;
    public float length;
    
    public OrientedPoint(Vector3 position, Quaternion rotation, float length)
    {
        this.position = position;
        this.rotation = rotation;
        this.length = length;
    }

    public Vector3 LocalToWorld(Vector3 point)
    {
        return position + rotation * point;
    }
    public Vector3 WorldToLocal(Vector3 point)
    {
        return Quaternion.Inverse(rotation) * (point - position);
    }

    public Vector3 LocalToWorldDirection(Vector3 dir)
    {
        return rotation * dir;
    }
}











//Vector3 currentForward = arm[0].LocalToWorld(Vector3.forward); //Direction of Oriented Point's forward 

////Vector3 currentUp = arm[0].LocalToWorld(Vector3.up);

////Vector3 currentRight = arm[0].LocalToWorld(Vector3.right);

//float constrainAngle = Mathf.Lerp(0, Mathf.PI / 2, constrain);
//float mag = 1 / Mathf.Sin(constrainAngle);
//Vector3 constrainVector = new Vector3(Mathf.Cos(constrainAngle), 0, Mathf.Sin(constrainAngle)).normalized * mag;
//float constrainProjectedX = Vector3.Dot(Vector3.right, constrainVector);
//float constrainProjectedY = Vector3.Dot(Vector3.right, constrainVector);
//Vector3 currentRight = arm[0].LocalToWorld(Vector3.forward + Vector3.right * constrainProjectedX);
//Vector3 currentUp = arm[0].LocalToWorld(Vector3.forward + Vector3.up * constrainProjectedY);
//float constraintRadius = Vector3.Distance(currentForward, currentRight);
////float y = currentUp.magnitude;
////float x = currentRight.magnitude;
//Gizmos.color = Color.red;
//            //Debug.Log(arm[0].position);
//            Debug.Log(constrainVector);
//            Gizmos.DrawLine(arm[0].position, arm[0].LocalToWorld(constrainVector));
//            Gizmos.color = Color.yellow;
//            Gizmos.DrawLine(arm[0].position, currentRight); //the point where constrains intersect
//            Gizmos.DrawLine(arm[0].position, currentUp); //the point where constrains intersect



//            Vector3 lookAtTarget;
//float constrainedTargetProjectedX = Vector3.Dot(Vector3.right, arm[0].WorldToLocal(testTarget.transform.position - arm[0].position));
//float constrainedTargetProjectedY = Vector3.Dot(Vector3.up, arm[0].WorldToLocal(testTarget.transform.position - arm[0].position));
//Vector3 constrainedTargetX = Vector3.forward + Vector3.right * constrainedTargetProjectedX;
//Vector3 constrainedTargetY = Vector3.forward + Vector3.up * constrainedTargetProjectedY;
//Vector3 constrainedTargetXYZ = constrainedTargetX * 0.5f + constrainedTargetY * 0.5f;
//constrainedTargetXYZ = arm[0].LocalToWorld(constrainedTargetXYZ);
//float distance = Vector3.Distance(constrainedTargetXYZ, currentForward);
//            if (distance > constraintRadius)
//            {
//                lookAtTarget = Vector3.Normalize(constrainedTargetXYZ - currentForward) * constraintRadius + currentForward;
//                Gizmos.color = Color.white;
//                testTarget.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
//            }
//            else
//            {
//                lookAtTarget = constrainedTargetXYZ;
//                Gizmos.color = Color.blue;
//                testTarget.transform.localScale = new Vector3(0.4f, 0.4f, 0.4f);
//            }


//            //Gizmos.DrawLine(arm[0].position, constrainedTargetX); //the point where constrains intersect
//            //Gizmos.DrawLine(arm[0].position, constrainedTargetY); //the point where constrains intersect
//            Gizmos.DrawWireCube(arm[0].position, Vector3.one* 2);

//            Gizmos.color = Color.black;
//            Gizmos.DrawLine(arm[0].position, lookAtTarget);