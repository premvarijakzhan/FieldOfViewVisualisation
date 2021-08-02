using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FieldOfView : MonoBehaviour
{
    public float viewRadius;

    [Range(0, 360)]
    public float viewAngle;

    public LayerMask targetMask;
    public LayerMask obstacleMask;

    [HideInInspector]
    public List<Transform> visibleTargets = new List<Transform>();

    public float meshResolution; //number rays cast per degree 

    public int edgeResolveIteration;

    public float edgeDistanceThreshold;

    public MeshFilter viewMeshFilter;
    Mesh viewMesh;


    void Start()
    {

        viewMesh = new Mesh();
        viewMesh.name = "View Mesh";
        viewMeshFilter.mesh = viewMesh;


        StartCoroutine(FindTargetsWithDelay(.2f));
    }

    IEnumerator FindTargetsWithDelay(float delay)
    {
        while (true)
        {
            yield return new WaitForSeconds(delay);
            FindVisibleTargets();
        }


    }



    private void LateUpdate()
    {
        DrawFieldOfView();
    }

    void FindVisibleTargets()
    {
        visibleTargets.Clear();

        Collider[] targetsInViewRadius = Physics.OverlapSphere(transform.position, viewRadius, targetMask);

        for (int i = 0; i < targetsInViewRadius.Length; i++)
        {
            Transform target = targetsInViewRadius[i].transform;
            Vector3 directionToTarget = (target.position - transform.position).normalized;
            if (Vector3.Angle(transform.forward, directionToTarget) < viewAngle / 2)
            {
                float distanceToTarget = Vector3.Distance(transform.position, target.position);

                if (!Physics.Raycast(transform.position, directionToTarget, distanceToTarget, obstacleMask))
                {
                    visibleTargets.Add(target);
                }
            }
        }

    }


    void DrawFieldOfView()
    {
        int rayCount = Mathf.RoundToInt(viewAngle * meshResolution);

        //how many degrees in each ray
        float rayAngleSize = viewAngle / rayCount;

        //maintain a list of all points that our view casts hits so that we can actually constrcut the mesh
        List<Vector3> viewPoints = new List<Vector3>();

        ViewCastInfo oldViewCast = new ViewCastInfo(); //find out if previous viewcast has hit an obstacle or not

        //loop through each of the rayCount
        for (int i = 0; i <= rayCount; i++) //NEED TO DO RAYCAST FOR EACH OF THE ANGLES IN CASE THERE;S AN OBSTACLE IN THE WAY
        {
            //current angle we're working with = object's current rotation - rotate back to left most view angle rotate clockwise in increments of the ray angle size until we get to the rightmost view angle
            float angle = transform.eulerAngles.y - viewAngle / 2 + rayAngleSize * i; //i = current rayCount

            // Debug.DrawLine(transform.position, transform.position + DirectionFromAngle(angle, true) * viewRadius, Color.red);
            ViewCastInfo newViewCastInfo = ViewCast(angle);

            if (i > 0)
            {
                bool edgeDistanceThresholdExceeded = Mathf.Abs(oldViewCast.distanceOfRay - newViewCastInfo.distanceOfRay) > edgeDistanceThreshold;

                if (oldViewCast.hit != newViewCastInfo.hit || (oldViewCast.hit && newViewCastInfo.hit && edgeDistanceThresholdExceeded)) //whethet or not to do edge detction
                {
                    EdgeInfo edge = FindEdge(oldViewCast, newViewCastInfo);
                    if (edge.pointA != Vector3.zero)
                    {
                        viewPoints.Add(edge.pointA);
                    }
                    if (edge.pointB != Vector3.zero)
                    {
                        viewPoints.Add(edge.pointB);
                    }
                }
            }

            viewPoints.Add(newViewCastInfo.point);
            oldViewCast = newViewCastInfo;
        }

        //viewmesh is the child of character objects hence all othe position 
        //of the vertices need to be local space, relative to the character

        //number of vertices = the number of viewPoint + origin Vertex
        int vertexCount = viewPoints.Count + 1;
        Vector3[] vertices = new Vector3[vertexCount]; //vertices with the size of vertex count
        int[] triangles = new int[(vertexCount - 2) * 3];

        //first vertex
        vertices[0] = Vector3.zero;

        //loop rest of the vertices and set them equal to the positions in our viewpoints list
        // minus 1 cause we already set up the first one
        for (int i = 0; i < vertexCount - 1; i++)
        {
            //i+1 so we wont overide the  first vertex
            vertices[i + 1] = transform.InverseTransformPoint(viewPoints[i]);

            //transform.InverseTransformPoint(viewPoints[i]) changes global point to local point relative 


            if (i < vertexCount - 2) //to not let go out of bounds of the array
            {

                //set triangles array; first vertex of each trinagle starts at the origin vertex
                triangles[i * 3] = 0;

                //the next vertex in the triangle
                triangles[i * 3 + 1] = i + 1;

                //the last vertex in the triangle
                triangles[i * 3 + 2] = i + 2;
            }
        }


        //redet everything in the viewmesh to start with
        viewMesh.Clear();
        viewMesh.vertices = vertices;
        viewMesh.triangles = triangles;
        viewMesh.RecalculateNormals();
    }

    EdgeInfo FindEdge(ViewCastInfo minViewCast, ViewCastInfo maxViewCast)
    {
        float minAngle = minViewCast.angle;
        float maxAngle = maxViewCast.angle;
        Vector3 minPoint = Vector3.zero;
        Vector3 maxPoint = Vector3.zero;

        for (int i = 0; i < edgeResolveIteration; i++)
        {
            float angle = (minAngle + maxAngle) / 2;
            ViewCastInfo newViewCast = ViewCast(angle);

            bool edgeDistanceThresholdExceeded = Mathf.Abs(minViewCast.distanceOfRay - newViewCast.distanceOfRay) > edgeDistanceThreshold;

            if (newViewCast.hit == minViewCast.hit && !edgeDistanceThresholdExceeded)
            {
                minAngle = angle;
                minPoint = newViewCast.point;

            }
            else
            {
                maxAngle = angle;
                maxPoint = newViewCast.point;
            }
        }

        return new EdgeInfo(minPoint, maxPoint);
    }

    //viewcast method will return a viewcastinfo struct
    //method to handle the raycast info for us so dont have to write each time up

    ViewCastInfo ViewCast(float globalAngle)
    {
        Vector3 direction = DirectionFromAngle(globalAngle, true);
        RaycastHit hit;

        if (Physics.Raycast(transform.position, direction, out hit, viewRadius, obstacleMask))
        {
            return new ViewCastInfo(true, hit.point, hit.distance, globalAngle);
        }
        else
        {
            return new ViewCastInfo(false, transform.position + direction * viewRadius, viewRadius, globalAngle);

        }

    }


    public Vector3 DirectionFromAngle(float angleInDegrees, bool angleIsGlobal)
    {
        if (!angleIsGlobal)
        {
            angleInDegrees += transform.eulerAngles.y;
        }
        return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad), 0, Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
    }

    //to handle the raycast info for us so dont have to write each time up

    public struct ViewCastInfo
    {
        public bool hit; //did it hit something
        public Vector3 point; //endpoint of the ray
        public float distanceOfRay; //length of the ray
        public float angle;//the angle of the ray was fired at 


        public ViewCastInfo(bool _hit, Vector3 _point, float _distanceOfTheRay, float _angle)
        {
            hit = _hit;
            point = _point;
            distanceOfRay = _distanceOfTheRay;
            angle = _angle;
        }
    }

    public struct EdgeInfo
    {
        public Vector3 pointA;
        public Vector3 pointB;

        public EdgeInfo(Vector3 _pointA, Vector3 _pointB)
        {
            pointA = _pointA;
            pointB = _pointB;
        }
    }

}
