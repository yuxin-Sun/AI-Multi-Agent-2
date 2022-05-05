using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{

    private DroneController m_Drone; // the car controller we want to use

    public GameObject my_goal_object;
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    Rigidbody my_rigidbody;

    public GameObject[] friends; // use these to avoid collisions
    public List<Vector3> locations;
    //Planing class
    PathPlanner path_planner;
    GraphEmbedding graph_embedding;
    TSP_graph graph;

    //Pathing variables
    int own_index;
    public List<ppc_pd_ray> drivers;
    public int current_driver;
    List<Vector3> my_path = new List<Vector3>();
    Vector3 position_;
    Vector3 velocity_;
    Vector3 goal_position;

    //RVO variables
    private AgentRVO myAgent_;
    private IList<AgentRVO> friendagents_;
    //private IList<Obstacle> obstacles_;


    private void Start()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

        // set hyperparameters
        float padding = 1F;


        //GameObject
        friends = GameObject.FindGameObjectsWithTag("Drone");
        my_rigidbody = GetComponent<Rigidbody>();
        goal_position = my_goal_object.transform.position;
        myAgent_ = new AgentRVO();

        locations = new List<Vector3>();
        for (int i = 0; i < friends.Length; i++)
        {
            locations.Add(friends[i].transform.position);
        }

        // find own index
        float min_distance = 100000F;
        own_index = -1;
        for (int i = 0; i < locations.Count; i++)
        {
            if ((locations[i] - transform.position).magnitude < min_distance)
            {
                min_distance = (locations[i] - transform.position).magnitude;
                own_index = i;
            }
        }
        //graph_embedding = new GraphEmbedding(terrain_manager.terrain_filename, 1, padding);
        //path_planner = new PathPlanner();

        //Create the viewpoints
        //my_path = path_planner.plan_path(start_pos,goal_pos, graph_embedding, terrain_manager.myInfo, 1);
        Vector3 start_pos = transform.position;
        Vector3 goal_pos = goal_position;
        my_path.Add(start_pos);
        my_path.Add(goal_pos);

        // Plot your path to see if it makes sense
        /*Vector3 old_wp = start_pos;
        foreach (var wp in my_path)
        {
            Debug.DrawLine(old_wp, wp, Color.blue, 100f);
            old_wp = wp;
        }
        */

    }


    private void FixedUpdate()
    {

        //Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z), Color.white, 1f);
        position_ = transform.position;
        velocity_ = my_rigidbody.velocity;
        Vector3 prevelocity_ = (goal_position - position_) / Time.deltaTime;
        // Debug.DrawLine(position_, position_ + prevelocity_, Color.red);
        myAgent_.setAgent(1f, 1f, 3F, 20, velocity_, position_);
        myAgent_.setprefVelocity_(prevelocity_);

        //update Agent-Neighbor's statement
        friends = GameObject.FindGameObjectsWithTag("Drone");
        myAgent_.clearNeighbor();
        for (int m = 0; m < friends.Length; m++)
        {
            if (m == own_index) continue;
            AgentRVO neighbor = new AgentRVO();
            Vector3 V_Neighbor = friends[m].GetComponent<Rigidbody>().velocity;
            Vector3 P_Neighbor = friends[m].transform.position;
            //Debug.Log("neighbor velocity:" + V_Neighbor + "for index:" + own_index);
            neighbor.setAgent(1f, 1f, 3F, 20, V_Neighbor, P_Neighbor);
            myAgent_.addNeighbor(neighbor);
        }

        // check range
        Vector3 relVect;
        if (myAgent_.minumneighborDist_ < 20f)  //set to 10
        {
            //get new relative velocity
            //Debug.Log("RVO applied");
            relVect = myAgent_.updateVelocity();
            if (velocity_.magnitude > 10.0f)
            {
                relVect = -relVect;
            }
        }
        else
        {
            Debug.Log("Derict");
            relVect = prevelocity_;
        }

        Debug.DrawLine(transform.position, transform.position + relVect, Color.white);
        // Debug.Log("aiming velocity:" + relVect + "for index:" + own_index);
        m_Drone.Move_vect(relVect);

        
    }



    // Update is called once per frame
    void Update()
    {

    }



}
