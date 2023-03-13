using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.Barracuda;
using Unity.MLAgentsExamples;
using Unity.VisualScripting;
using System;

[RequireComponent(typeof(AGVController))]
[RequireComponent(typeof(DirectIndictor))]

public class AGVAgent : Agent
{
    //[HideInInspector]
    public AGVController _aGVController;
    public SimulationManager _simulationManager;
    public DirectIndictor _directIndictor;
    public CircumstanceControl envController;



    BufferSensorComponent BufferSensor;                //接收器
    private float[] listOfInfo;                        //接收信息集合
    public List<AGVAgent> Cars;                       //用于AGV集合
    public List<GameObject> Walls;
    public float Bound = 50.0f;                        //动态监测范围
    public float AGVcollisionPenalty = -0f;
    public float WallcollisionPenalty = -0f;
    public float ParkingLinePenalty = -20f;
    public float TargetZoneRate;



    Rigidbody rBody_Agv;

    public GameObject Target;
    public Rigidbody Container;
    public GameObject _container;
    public GameObject Tuoban;
    public GameObject testarea;
    //public GameObject wallf, wallb, walll, wallr;

    public WheelCollider lf, rf, lb, rb;
    public Transform lfT, rfT, lbT, rbT, forleft, forright, bacleft, bacright;

    public float disToTarget;
    public float newdisToTarget;
    public float olddisToTarget;

    public float maxX, minX, maxZ, minZ;
    public float MaxNum = 100000000f;


    public float brakeRate = 10000f;
    public float maxSteeringAngle = 30f; //转向系数
    public float powerRate = 1000f; //推进系数
    public float minmasscontainer = 2500f;
    public float maxmasscontainer = 4000f;
    public float mindistotarget = 20f;
    public Vector3[] veces;

    public float x, y, z, xr, yr, zr;


    void Start()
    {
        envController = this.transform.parent.gameObject.GetComponent<CircumstanceControl>();
        //Time.timeScale = 5f;
    }



    public override void Initialize()
    {
        rBody_Agv = GetComponent<Rigidbody>();
        BufferSensor = this.GetComponent<BufferSensorComponent>();
        _aGVController = GetComponent<AGVController>();
        _aGVController.agv = this;
        _simulationManager = GetComponent<SimulationManager>();
        _simulationManager.agv = this;
        _directIndictor = GetComponent<DirectIndictor>();
        _directIndictor.agv = this;

        x = this.transform.localPosition.x;  //获得初始化的位置
        y = this.transform.localPosition.y;
        z = this.transform.localPosition.z;
        xr = this.transform.localEulerAngles.x;
        yr = this.transform.localEulerAngles.y;
        zr = this.transform.localEulerAngles.z;

    }
    public override void OnEpisodeBegin()
    {
        //Target.transform.localPosition = new Vector3(Random.Range(-300f, -150f), 1f, Random.Range(-270f, -70f));

        this.transform.localPosition = new Vector3(x, y, z);
        this.transform.localRotation = Quaternion.Euler(0f, UnityEngine.Random.Range(0f, 360f), 0f);
        //this.transform.localRotation = Quaternion.Euler(0f, this.transform.localEulerAngles.y, 0f);
        //this.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
        //this.transform.localRotation = Quaternion.Euler(xr, yr, zr);
        this.GetComponent<Rigidbody>().velocity = Vector3.zero;
        this.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;

        Container.mass = UnityEngine.Random.Range(minmasscontainer, maxmasscontainer);
        Container.transform.localPosition = new Vector3(0f, 2f, 0f);
        Container.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
        Container.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        Container.GetComponent<Rigidbody>().velocity = Vector3.zero;

        Vector3 vecto2 = Vector3.zero;
        Vector3 vecfrom2 = Vector3.zero;

        //this.Target.transform.localPosition = new Vector3(Random.Range(this.minX, this.maxX) * TargetZoneRate, 1f, Random.Range(this.minZ, this.maxZ) * TargetZoneRate);

        disToTarget = this.GetRealDisFromAGVtoCollider(this, this.Target.GetComponent<BoxCollider>(), ref vecto2, ref vecfrom2);


    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation((Target.transform.localPosition.x - minX) / (maxX - minX));
        sensor.AddObservation((Target.transform.localPosition.z - minZ) / (maxZ - minZ));
        sensor.AddObservation((this.transform.localPosition.x - minX) / (maxX - minX));
        sensor.AddObservation((this.transform.localPosition.z - minZ) / (maxZ - minZ));
        sensor.AddObservation((rBody_Agv.angularVelocity.x + 20f) / 40f);
        sensor.AddObservation((rBody_Agv.angularVelocity.y + 20f) / 40f);
        sensor.AddObservation((rBody_Agv.angularVelocity.z + 20f) / 40f);
        sensor.AddObservation((rBody_Agv.velocity.x + 50f) / 100f);
        sensor.AddObservation((rBody_Agv.velocity.z + 50f) / 100f);
        sensor.AddObservation((rBody_Agv.GetComponent<Transform>().rotation.eulerAngles.x + 180f) / 360f);
        sensor.AddObservation((rBody_Agv.GetComponent<Transform>().rotation.eulerAngles.y + 180f) / 360f);
        sensor.AddObservation((rBody_Agv.GetComponent<Transform>().rotation.eulerAngles.z + 180f) / 360f);
        sensor.AddObservation((Container.mass - minmasscontainer) / (maxmasscontainer - minmasscontainer));
        sensor.AddObservation((_directIndictor.directIndictor.transform.rotation.eulerAngles.y + 180f) / 360f);
        sensor.AddObservation( (float)Math.Sqrt( disToTarget*disToTarget / ( (maxX - minX) * (maxX - minX) + (maxZ - minZ) *(maxZ - minZ) ) ));

        //sensor.AddObservation((lf.steerAngle + 1f) / 2f);
        //sensor.AddObservation((lb.steerAngle + 1f) / 2f);

        //sensor.AddObservation((lf.motorTorque + this.powerRate) / (2f * this.powerRate));
        //sensor.AddObservation((lf.brakeTorque + this.brakeRate) / (2f * this.brakeRate));

        //sensor.AddObservation(_container.transform.localPosition);
        //sensor.AddObservation(_container.transform.localRotation);

        //动态监测
        //CheckAndAdd();
    }

    //动态监测Attention机制是否正常
    public void CheckAndAdd()
    {
        //float disscore;
        foreach (AGVAgent car in Cars)
        {
            Vector3 vecfrom = Vector3.zero;
            Vector3 vecto = Vector3.zero;
            float dis = this.GetRealDisbetweenAGVs(this, car, ref vecfrom, ref vecto);
            if (dis <= Bound)
            {
                AddInfo(car);


                //Debug.DrawLine(vecfrom, vecto, Color.yellow);
            }
        }

        foreach (GameObject wall in Walls)
        {
            Vector3 vecto = Vector3.zero;
            Vector3 vecfrom = Vector3.zero;
            float dis1 = this.GetRealDisFromAGVtoCollider(this, wall.GetComponent<BoxCollider>(), ref vecto, ref vecfrom);
            if (dis1 <= Bound)
            {
                AddInfoWalls(wall, dis1, vecto, vecfrom);



            }
        }

        Vector3 vecto1 = Vector3.zero;
        Vector3 vecfrom1 = Vector3.zero;
        float dis2 = GetRealDisFromAGVtoCollider(this, Target.GetComponent<BoxCollider>(), ref vecto1, ref vecfrom1);
        //float dis2 = Vector3.Distance(this.transform.localPosition,Target.transform.localPosition);
        if (dis2 <= Bound)
        {
            AddInfoTarget(dis2);
            //Debug.DrawLine(vecfrom1, vecto1, Color.yellow);

        }

    }

    // Attention机制的信息的采集接口
    public void AddInfo(AGVAgent Car)
    {
        //将范围内的每个AGV的信息转换

        Vector3 vecfrom = Vector3.zero;
        Vector3 vecto = Vector3.zero;

        listOfInfo = new float[27];
        listOfInfo[0] = (Car.transform.localPosition.x - minX) / (maxX - minX);                        //转换位置信息，使得输入为0~1
        listOfInfo[1] = (Car.transform.localPosition.z - minZ) / (maxZ - minZ);                          //转换位置信息，使得输入为0~1
        listOfInfo[2] = (Car.GetComponent<Rigidbody>().velocity.x + 50f) / 100f;           //转换位置信息，使得输入为0~1
        listOfInfo[3] = (Car.GetComponent<Rigidbody>().velocity.z + 50f) / 100f;           //转换位置信息，使得输入为0~1
        listOfInfo[4] = (Vector3.Dot(Vector3.Normalize(this.transform.forward), Vector3.Normalize(Car.transform.forward)) + 1f) / 2.000001f;

        if (System.Math.Abs(listOfInfo[4]) <= 0.000001f) // 有时内积计算可能存在误差，所以需要调整，以确保相关参数介于0和1之间
        {
            listOfInfo[4] = 0f;
        }
        else if (System.Math.Abs(listOfInfo[4]) - 1 >= 0.000001f)
        {
            listOfInfo[4] = 1f;
        }

        listOfInfo[4] = (Vector3.Dot(Vector3.Normalize(this.transform.forward), Vector3.Normalize(Car.transform.forward)) + 1f) / 2.000001f;
        listOfInfo[5] = (Car.GetComponent<Rigidbody>().angularVelocity.x + 20f) / 40f;
        listOfInfo[6] = (Car.GetComponent<Rigidbody>().angularVelocity.y + 20f) / 40f;
        listOfInfo[7] = (Car.GetComponent<Rigidbody>().angularVelocity.z + 20f) / 40f;
        listOfInfo[8] = this.GetRealDisbetweenAGVs(this, Car, ref vecfrom, ref vecto) / Bound;

        if ( listOfInfo[8]*Bound < 10f && listOfInfo[8] * Bound > 0f)
        {
            this.AddReward( -0.001f/ ( listOfInfo[8] * Bound + 0.02f)  );
        }


        listOfInfo[9] = (Car.lf.motorTorque + Car.powerRate) / (2 * Car.powerRate);
        listOfInfo[10] = Car.lf.brakeTorque / Car.brakeRate;


        listOfInfo[11] = 0f;
        listOfInfo[12] = 0f;


        listOfInfo[13] = (Car._aGVController.CurrentFrontSteeringAngle + 1f) / 2f;
        listOfInfo[14] = (Car._aGVController.CurrentBackSteeringAngle + 1f) / 2f;
        listOfInfo[15] = (Car.Container.mass - minmasscontainer) / (maxmasscontainer - minmasscontainer);
        listOfInfo[16] = Vector3.Distance(this.forright.position, Car.forright.position) / (2 * Bound);
        listOfInfo[17] = Vector3.Distance(this.forright.position, Car.forleft.position) / (2 * Bound);
        listOfInfo[18] = Vector3.Distance(this.forright.position, Car.bacright.position) / (2 * Bound);
        listOfInfo[19] = Vector3.Distance(this.forright.position, Car.bacleft.position) / (2 * Bound);
        listOfInfo[20] = Vector3.Distance(this.forleft.position, Car.forright.position) / (2 * Bound);
        listOfInfo[21] = Vector3.Distance(this.forleft.position, Car.forleft.position) / (2 * Bound);
        listOfInfo[22] = Vector3.Distance(this.forleft.position, Car.bacright.position) / (2 * Bound);
        listOfInfo[23] = Vector3.Distance(this.forleft.position, Car.bacleft.position) / (2 * Bound);
        listOfInfo[24] = (Car.Target.transform.localPosition.x - minX) / (maxX - minX);
        listOfInfo[26] = (Car.Target.transform.localPosition.z - minZ) / (maxZ - minZ);
        listOfInfo[25] = (Car.transform.rotation.y + 180f) / 360f;


        for (int i = 0; i < listOfInfo.Length; i++)
        {
            if ((listOfInfo[i] > 1.0000001f) || (listOfInfo[i] < -0.0000001f))
            {
                Debug.Log("something is wrong!");
                Debug.Log(i);
            }
        }

        BufferSensor.AppendObservation(listOfInfo);

    }
    public void AddInfoWalls(GameObject wall, float dis, Vector3 vecto, Vector3 vecfrom)
    {
        //将范围内的每个AGV的信息转换
        listOfInfo = new float[27];
        listOfInfo[0] = (vecto.x - minX) / (maxX - minX);                         //转换位置信息，使得输入为0~1
        listOfInfo[1] = (vecto.z - minZ) / (maxZ - minZ);                        //转换位置信息，使得输入为0~1
        listOfInfo[2] = 0f;                        //转换位置信息，使得输入为0~1
        listOfInfo[3] = 0f;                        //转换位置信息，使得输入为0~1
        listOfInfo[4] = 0f;
        listOfInfo[5] = 0f;
        listOfInfo[6] = 0f;
        listOfInfo[7] = 0f;
        listOfInfo[8] = dis / Bound;
       // if ( dis < 5f && dis > 0f)
       // {
       //     this.AddReward(-0.01f / (dis + 10f));
       // }


        listOfInfo[9] = 0f;
        listOfInfo[10] = 0f;

        listOfInfo[11] = 0f;
        listOfInfo[12] = 1f;
        
        listOfInfo[13] = 0f;
        listOfInfo[14] = 0f;
        listOfInfo[15] = 0f;
        listOfInfo[16] = 0f;
        listOfInfo[17] = 0f;
        listOfInfo[18] = 0f;
        listOfInfo[19] = 0f;
        listOfInfo[20] = 0f;
        listOfInfo[21] = 0f;
        listOfInfo[22] = 0f;
        listOfInfo[23] = 0f;
        listOfInfo[24] = 0f;
        listOfInfo[25] = 0f;
        listOfInfo[26] = 0f;

        BufferSensor.AppendObservation(listOfInfo);

        //Debug.DrawLine(vecfrom, vecto, Color.white);


    }
    public void AddInfoTarget(float dis)
    {
        listOfInfo = new float[27];
        listOfInfo[0] = (this.Target.transform.localPosition.x - minX) / (maxX - minX);                         //转换位置信息，使得输入为0~1
        listOfInfo[1] = (this.Target.transform.localPosition.z - minZ) / (maxZ - minZ);                         //转换位置信息，使得输入为0~1
        listOfInfo[2] = 0f;
        listOfInfo[3] = 0f;
        listOfInfo[4] = (Vector3.Dot(Vector3.Normalize(this.transform.forward), Vector3.Normalize(this._directIndictor.directIndictor.transform.forward)) + 1) / 2;
        listOfInfo[5] = 0f;
        listOfInfo[6] = 0f;
        listOfInfo[7] = 0f;
        listOfInfo[8] = dis / Bound;
        listOfInfo[9] = 0f;
        listOfInfo[10] = 0f;

        listOfInfo[11] = 1f;
        listOfInfo[12] = 1f;

        listOfInfo[13] = 0f;
        listOfInfo[14] = 0f;
        listOfInfo[15] = 0f;
        listOfInfo[16] = 0f;
        listOfInfo[17] = 0f;
        listOfInfo[18] = 0f;
        listOfInfo[19] = 0f;
        listOfInfo[20] = 0f;
        listOfInfo[21] = 0f;
        listOfInfo[22] = 0f;
        listOfInfo[23] = 0f;
        listOfInfo[24] = 0f;
        listOfInfo[25] = 0f;
        listOfInfo[26] = 0f;


        BufferSensor.AppendObservation(listOfInfo);

    }


    public float GetAngle(Transform ob1, Transform ob2)
    {
        //以ob1为当前AGV：
        Vector3 v1 = ob1.right;
        Vector3 v2 = new Vector3(ob2.localPosition.x - ob1.localPosition.x, 0, ob2.localPosition.z - ob1.localPosition.z);
        float angle = Vector3.Angle(v1, v2);
        float dot = Vector3.Dot(ob1.forward, v2);

        if (dot < 0)
        {
            angle = 360f - angle;
        }
        //Debug.Log("角度是："+ angle);
        return angle;
    }

    //AI对车辆的控制接口
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        _aGVController.CurrentFrontSteeringAngle = actionBuffers.ContinuousActions[0];
        _aGVController.CurrentAcceleration = actionBuffers.ContinuousActions[1];
        _aGVController.CurrentBackSteeringAngle = actionBuffers.ContinuousActions[2];
        //_aGVController.CurrentBrakeTorque        = actionBuffers.DiscreteActions[0];
        _aGVController.CurrentBrakeTorque = actionBuffers.ContinuousActions[3];
    }

    //手动控制的接口
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.ContinuousActions;
        discreteActionsOut[0] = Input.GetAxis("Horizontal");
        discreteActionsOut[1] = Input.GetAxis("Vertical");

        //var discreteActionsOut2 = actionsOut.DiscreteActions;
        // discreteActionsOut2[0] = (int)Input.GetAxis("Jump");

    }

    //碰撞机制
    private void OnTriggerEnter(Collider other) //停车线
    {
        if (other.gameObject.CompareTag("ParkingLine"))
        {
            Debug.Log("触碰停车线");
            //AddReward(ParkingLinePenalty);
            EndEpisode();
        }
    }
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("AGV"))
        {
            AddReward(-200f * Vector3.Distance(this.GetComponent<Rigidbody>().velocity, Vector3.zero));
            //envController.m_AGVAgentGroup.AddGroupReward(-2f);
            //this.EndEpisode();
            Debug.Log("触碰AGV");
        }
        if (collision.gameObject.CompareTag("Walls"))
        {
            AddReward(-2f);
            this.EndEpisode();
            Debug.Log("触碰Wall");
            //envController.m_AGVAgentGroup.AddGroupReward(-2f);
        }

        //Debug.Log(this._aGVController.CurrentBrakeTorque);
    }
    private void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.CompareTag("AGV"))
        {
            AddReward(-0.1f);
        }
        //if (collision.gameObject.CompareTag("Walls"))
        {
            //AddReward(-0.01f);
            //this.EndEpisode();
            //Debug.Log("触碰Wall");
        }
    }


    //随时需要运行的代码：
    void FixedUpdate()
    {

        this.Updatedispoint(); //更新四个顶点的坐标
        maxX = this.transform.parent.gameObject.GetComponent<CircumstanceControl>().maxX;
        minX = this.transform.parent.gameObject.GetComponent<CircumstanceControl>().minX;
        maxZ = this.transform.parent.gameObject.GetComponent<CircumstanceControl>().maxZ;
        minZ = this.transform.parent.gameObject.GetComponent<CircumstanceControl>().minZ;

        if (this.disToTarget < this.mindistotarget)
        {
            this.AddReward(5f);
            //envController.m_AGVAgentGroup.AddGroupReward(10f);

            //this.Target.transform.localPosition = new Vector3(Random.Range(this.minX, this.maxX) * TargetZoneRate, 1f, Random.Range(this.minZ, this.maxZ) * TargetZoneRate);
        }


        CheckAndAdd();

    }




    //计算AGV之间的精确距离
    public float GetRealDisFromAGVtoCollider(AGVAgent agv, BoxCollider boxCollider, ref Vector3 closestpointvecto, ref Vector3 closestpointvecfrom)//返回AGV距离BoxCollider的实际距离（考虑Collider的形状影响）
    {
        List<Vector3> vecsetto = new List<Vector3>();
        List<Vector3> vecsetfrom = new List<Vector3>();
        var vecfl = boxCollider.ClosestPoint(agv.forleft.transform.position);
        vecsetto.Add(vecfl);
        vecsetfrom.Add(agv.forleft.transform.position);

        var vecfr = boxCollider.ClosestPoint(agv.forright.transform.position);
        vecsetto.Add(vecfr);
        vecsetfrom.Add(agv.forright.transform.position);

        var vecbl = boxCollider.ClosestPoint(agv.bacleft.transform.position);
        vecsetto.Add(vecbl);
        vecsetfrom.Add(agv.bacleft.transform.position);

        var vecbr = boxCollider.ClosestPoint(agv.bacright.transform.position);
        vecsetto.Add(vecbr);
        vecsetfrom.Add(agv.bacright.transform.position);

        float[] listOfDis = new float[4];
        listOfDis[0] = Vector3.Distance(agv.forleft.transform.position, vecfl);
        listOfDis[1] = Vector3.Distance(agv.forright.transform.position, vecfr);
        listOfDis[2] = Vector3.Distance(agv.bacleft.transform.position, vecbl);
        listOfDis[3] = Vector3.Distance(agv.bacright.transform.position, vecbr);



        float tempdis = MaxNum;

        for (int i = 0; i < listOfDis.Length; i++)
        {
            if (listOfDis[i] <= tempdis)
            {
                tempdis = listOfDis[i];
                closestpointvecto = vecsetto[i];
                closestpointvecfrom = vecsetfrom[i];
            }
        }
        return tempdis;
    }
    //计算AGV与Wall之间的准确距离
    public float GetRealDisbetweenAGVs(AGVAgent agv1, AGVAgent agv2, ref Vector3 vecfrom, ref Vector3 vecto)
    {
        Vector3 vec1to = Vector3.zero;
        Vector3 vec1from = Vector3.zero;
        Vector3 vec2to = Vector3.zero;
        Vector3 vec2from = Vector3.zero;

        float dis1 = GetRealDisFromAGVtoCollider(agv1, agv2.GetComponent<BoxCollider>(), ref vec1to, ref vec1from);
        float dis2 = GetRealDisFromAGVtoCollider(agv2, agv1.GetComponent<BoxCollider>(), ref vec2to, ref vec2from);
        float dis3 = 0f;

        if (dis1 >= dis2)
        {
            dis3 = dis2;
            vecfrom = vec2from;
            vecto = vec2to;

        }
        else
        {
            dis3 = dis1;
            vecfrom = vec1from;
            vecto = vec1to;
        }

        return dis3;
    }

    //服务上述几个功能的辅助函数
    Vector3[] GetBoxColliderVertexPositions(BoxCollider boxcollider)
    {
        var vertices = new Vector3[8];
        //下面4个点
        vertices[0] = boxcollider.transform.TransformPoint(boxcollider.center + new Vector3(boxcollider.size.x, -boxcollider.size.y, boxcollider.size.z) * 0.5f);
        vertices[1] = boxcollider.transform.TransformPoint(boxcollider.center + new Vector3(-boxcollider.size.x, -boxcollider.size.y, boxcollider.size.z) * 0.5f);
        vertices[2] = boxcollider.transform.TransformPoint(boxcollider.center + new Vector3(-boxcollider.size.x, -boxcollider.size.y, -boxcollider.size.z) * 0.5f);
        vertices[3] = boxcollider.transform.TransformPoint(boxcollider.center + new Vector3(boxcollider.size.x, -boxcollider.size.y, -boxcollider.size.z) * 0.5f);
        //上面4个点
        vertices[4] = boxcollider.transform.TransformPoint(boxcollider.center + new Vector3(boxcollider.size.x, boxcollider.size.y, boxcollider.size.z) * 0.5f);
        vertices[5] = boxcollider.transform.TransformPoint(boxcollider.center + new Vector3(-boxcollider.size.x, boxcollider.size.y, boxcollider.size.z) * 0.5f);
        vertices[6] = boxcollider.transform.TransformPoint(boxcollider.center + new Vector3(-boxcollider.size.x, boxcollider.size.y, -boxcollider.size.z) * 0.5f);
        vertices[7] = boxcollider.transform.TransformPoint(boxcollider.center + new Vector3(boxcollider.size.x, boxcollider.size.y, -boxcollider.size.z) * 0.5f);
        return vertices;
    }
    private void Updatedispoint()
    {

        this.veces = GetBoxColliderVertexPositions(this.GetComponent<BoxCollider>());
        this.forright.transform.position = this.veces[4];
        this.forleft.transform.position = this.veces[5];
        this.bacleft.transform.position = this.veces[6];
        this.bacright.transform.position = this.veces[7];

    }

}