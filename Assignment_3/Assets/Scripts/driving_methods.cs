using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// this script contains driving methods such that the main carAI won't become a mess

// the idea: every 'planner' object gets 
// INPUT path (as list of Vector3), terrain, and all else needed, 
// OUTPUTS driving commands in every timestep

public class collision_avoider
{

    public float strength, far_strength, cutoff_distance;
    public float degree;

    public collision_avoider(float strength, float far_strength, float cutoff_distance, float degree)
    {
        this.strength = strength;
        this.far_strength = far_strength;
        this.cutoff_distance = cutoff_distance;
        this.degree = degree;
    }

    public float repulsion_megnitude(float distance)
    {

        if (distance > this.cutoff_distance)
        {
            return this.far_strength * (1 - distance / this.cutoff_distance);
        }

        return this.strength * Mathf.Pow(1 - distance / this.cutoff_distance, this.degree);
    }

    public Vector3 full_acceleration(Vector3 own_position, List<Vector3> positions)
    {

        Vector3 full = new Vector3(0, 0, 0);

        foreach (var position in positions)
        {
            if ((own_position - position).magnitude > 0.001f)
            { // ie. it is not the own position
                full = full + (own_position - position).normalized * this.repulsion_megnitude((own_position - position).magnitude);
            }
        }

        return full;
    }

    public List<float> driving_command(Vector3 own_position, List<Vector3> positions, Vector3 right, Vector3 forward)
    {

        Vector3 desired_acceleration = this.full_acceleration(own_position, positions);

        float steering = Vector3.Dot(desired_acceleration, right);
        float acceleration = Vector3.Dot(desired_acceleration, forward);

        float[] commands = { steering, acceleration, acceleration, 0f };

        return new List<float>(commands);
    }
}


// Ernest, Peter method of driving, PPC, PD, raycasts and finite automata

public class ppc_pd_ray
{
    TerrainManager terrain_manager;
    public PP_controller control3, control_backup;
    public List<RayController> rays;
    public int state, init_state;
    public bool forward, right, brake_at_start;
    public float trap_forward;
    public List<Vector3> waypoints;
    public Vector3 goal_customer;
    public Vector3 direction;
    public float avg_speed, tau_speed, avg_speed_threshold, timer, timer_tau, ray_threshold, ray_k_r;
    public float stuck_radius, padding, vmax, k_d, v_tent;


    public ppc_pd_ray(float padding, float stuck_radius, float avg_speed_threshold, float tau_speed, float timer_tau,
                      float ray_threshold, float ray_k_r, List<Vector3> waypoints, TerrainManager terr_mngr, float vmax = 1000f, bool brake_at_start = false, float mu = 0.3f)
    {

        this.padding = padding;
        this.stuck_radius = stuck_radius;
        this.avg_speed_threshold = avg_speed_threshold;
        this.tau_speed = tau_speed;
        this.timer_tau = timer_tau;
        this.waypoints = waypoints;
        this.ray_threshold = ray_threshold;
        this.ray_k_r = ray_k_r;

        this.state = 0;
        this.avg_speed = 5F;
        this.timer = 100F;
        this.forward = true;
        this.trap_forward = 1F;
        this.right = true;
        this.direction = new Vector3();
        this.terrain_manager = terr_mngr;
        this.brake_at_start = brake_at_start;
        this.vmax = vmax;

        Debug.Log("wp count: " + this.waypoints.Count);

        this.goal_customer = this.waypoints[this.waypoints.Count - 1];




        // also initialize the path


        List<Vector2> waypoints2d = new List<Vector2>();
        foreach (var waypoint in this.waypoints)
        {
            waypoints2d.Add(new Vector2(waypoint[0], waypoint[2]));
        }

        Debug.Log("waypoint list created");

        polygon_path p1 = new polygon_path(new waypoint_list(waypoints2d), padding, terrain_manager.terrain_filename);
        polygon_path p2 = new polygon_path(p1);

        // generate path, smooth it and set speed limits, initielize PPC controllers 

        // signature:
        // public PP_controller(polygon_path _path, float _lookahead, float padding, float coarseness, float max_deviation, float k_p, float k_d, float v, float mu)

        this.k_d = 0.5f;

        if (this.vmax < 900f)
        {
            this.v_tent = 20f;
        }
        else
        {
            this.v_tent = 20f;
        }


        PP_controller pp_ctr3 = new PP_controller(p1, 15F, this.padding, 1F, 100000F, 1.5F, this.k_d, this.v_tent, mu, vmax);
        PP_controller pp_ctr_backup = new PP_controller(p2, 2.8F, this.padding, 1F, 100000F, 2.5F, this.k_d, this.v_tent, mu, vmax);

        this.control3 = pp_ctr3;
        this.control_backup = pp_ctr_backup;

        this.direction = this.control3.path.desired_driving_direction_3d(pp_ctr3.path.wp.waypoints[0]);

        // add ray controllers

        List<RayController> rays = new List<RayController>();

        float current_angle;

        for (int _ray = 0; _ray < 8; _ray++)
        {

            current_angle = (float)_ray * Mathf.PI * 2 / (float)8;
            Debug.Log("angle " + current_angle);

            if (_ray == 0 | _ray == 4)
            {
                continue;
            }

            if (_ray == 2 | _ray == 6)
            { // forward is z!!
                rays.Add(new RayController(new Vector3(Mathf.Cos(current_angle), 0, Mathf.Sin(current_angle)), this.ray_threshold, 15 * this.ray_k_r));
            }
            else if (_ray == 0 || _ray == 4)
            {
                rays.Add(new RayController(new Vector3(Mathf.Cos(current_angle), 0, Mathf.Sin(current_angle)), this.ray_threshold, 1.5F * this.ray_k_r));
            }
            else
            {
                rays.Add(new RayController(new Vector3(Mathf.Cos(current_angle), 0, Mathf.Sin(current_angle)), this.ray_threshold, 3F * this.ray_k_r));
            }
        }



        this.rays = rays;

        if (this.brake_at_start)
        {
            this.init_state = -1;
        }
        else
        {
            this.init_state = 1;
        }


        Debug.Log("init done for driver");

    }

    public List<float> driving_command(Vector3 position, Vector3 velocity, Vector3 forward, Vector3 right)
    {

        //***************************
        // synopsis: upon vehicle positional data, we compute the driving commands implied by the finite automaton method
        //***************************

        if (this.init_state == -1)
        {
            this.control3.vmax = 0f;
            this.control_backup.vmax = 0f;
            this.control3.k_d = 0.1f;
            this.control_backup.k_d = 0.1f;

            Debug.Log("initial brake state");

            if (velocity.magnitude < 0.5f)
            {
                this.init_state = 1;
                this.control3.vmax = this.vmax;
                this.control_backup.vmax = this.vmax;
                this.control3.k_d = this.k_d;
                this.control_backup.k_d = this.k_d;
            }
        }

        if (this.control3.path.last_visited == this.control3.path.wp.waypoints.Count)
        {
            this.control3.vmax = 0f;
            this.control_backup.vmax = 0f;

            Debug.Log("braking to 0");
        }

        List<float> command = new List<float>();

        Debug.Log(this.state);

        if (this.state == 0)
        {

            command = normal_driving(position, velocity, forward, right);

            // check if changing state is needed

            this.avg_speed = Time.fixedDeltaTime / this.tau_speed * velocity.magnitude + (1F - Time.fixedDeltaTime / this.tau_speed) * this.avg_speed;
            if (Vector3.Angle(forward, this.control_backup.path.desired_driving_direction_3d(position)) > 100F)
            {
                this.state = 1;
                this.avg_speed = 8F;
                this.right = (Vector3.SignedAngle(forward, this.control_backup.path.desired_driving_direction_3d(position), new Vector3(0, 1, 0)) > 0F);
                this.forward = true;
                this.direction = this.control_backup.path.desired_driving_direction_3d(position);
            }
            if (this.avg_speed < this.avg_speed_threshold)
            {
                this.state = 2;
                this.timer = 100;
            }

        }
        else if (this.state == 1)
        { // reorientation

            command = reorientation(this.direction, position, velocity, forward, right, 5.5F, 2F, this.right, this.forward);

            this.avg_speed = 1F / this.tau_speed / 4F * velocity.magnitude + (1F - 1F / this.tau_speed / 4F) * this.avg_speed; // slow down time artificially


            // switch driving direction?

            if (command.Count == 5)
            {
                if (command[4] > 0F) // switch direction
                {
                    this.forward = !(this.forward);
                }

            }

            // managing state transitions

            if (Vector3.Angle(forward, this.direction) < 35F)
            {
                this.state = 0;
                this.avg_speed = 5F;
            }
            if (this.avg_speed < 0.5F)
            {
                this.state = 2;
                this.timer = 100;
            }

        }
        else if (this.state == 2)
        { // the car is stuck

           // RaycastHit hit, hit2;


            if (this.timer > 50)
            {

                command = untrap_driving(position, velocity, forward, right, this.control_backup.path.find_closest_point3d_stuck(position), this.control_backup.k_p);
                //Debug.Log(this.timer);
                this.timer = this.timer * (1F - Time.fixedDeltaTime / this.timer_tau);
            }

            // manage state transitions

            else
            {
                this.state = 0; //!!!!!
                this.trap_forward = -1F * this.trap_forward;
                this.avg_speed = 8F; // prevent 1-2 to fro swithcing
                this.right = (Vector3.SignedAngle(forward, this.control3.path.desired_driving_direction_3d(position), new Vector3(0, 1, 0)) > 0F);
                this.forward = true;
                command = untrap_driving(position, velocity, forward, right, this.control_backup.path.find_closest_point3d(position), this.control_backup.k_p);

                if (Vector3.Angle(forward, this.control_backup.path.desired_driving_direction_3d(position)) > 50F)
                {
                    this.state = 1;
                    this.right = (Vector3.SignedAngle(forward, this.control_backup.path.desired_driving_direction_3d(position), new Vector3(0, 1, 0)) > 0F);
                    this.avg_speed = 8F;
                    this.forward = (this.trap_forward > 0F);
                    this.direction = this.control_backup.path.desired_driving_direction_3d(position);
                }
            }
        }

        List<float> _c = new List<float>();

        for (int i = 0; i < 4; i++)
        {
            _c.Add(command[i]);
        }

        return _c;
    }

    List<float> normal_driving(Vector3 position, Vector3 velocity, Vector3 forward, Vector3 right) // return driving command
    {
        Vector3 acc;
        acc = PPcontroller_acceleration(position, velocity, forward, right) + Ray_acceleration(position, velocity, forward, right, true);

        List<float> command;
        command = this.control3.acceleration_to_command_normal_car(acc, velocity, position, right, forward);

        return command;
    }

    List<float> reorientation(Vector3 direction, Vector3 position, Vector3 velocity, Vector3 forward, Vector3 right, float v, float k_d, bool b_right, bool b_forward) // reorient itself, goal (tiny) velocity and steering direction, return 0 if finished
    {
        List<float> command;

        Vector3 acc;
        if (b_forward)
        {
            acc = -1F * k_d * (velocity - forward * v);
        }
        else
        {
            acc = -1F * k_d * (velocity + forward * v);
        }

        Vector3 ray_acc = Ray_acceleration(position, velocity, forward, right, true);

        command = this.control3.acceleration_to_command_reorient_car(acc, velocity, position, right, forward);

        if (b_right)
        {
            if (b_forward)
            {
                command[0] = 1F;
            }
            else
            {
                command[0] = -1F;
            }
        }
        else
        {
            if (b_forward)
            {
                command[0] = -1F;
            }
            else
            {
                command[0] = 1F;
            }
        }


        if (ray_acc.magnitude > acc.magnitude * 4F)
        {// wall close, switch direction
            command.Add(1F); // bit clumsy but that's it
        }


        return command;
    }


    List<float> untrap_driving(Vector3 position, Vector3 velocity, Vector3 forward, Vector3 right, Vector3 closest, float kp) // push away from walls and then reorient itself
    {
        Vector3 acc;
        Vector3 car_forward = forward.normalized;


        if (Vector3.Dot(closest - position, car_forward) * this.trap_forward < 0F)
        {
            acc = kp * (closest - position);
            acc = acc - 2F * (acc - car_forward * Vector3.Dot(car_forward, acc));


        }
        else
        {
            acc = kp * (closest - position);
        }

        acc = acc + 3F * Ray_acceleration(position, velocity, forward, right, false);


        List<float> command;
        command = this.control3.acceleration_to_command_normal_car(acc, velocity, position, right, forward);

        return command;
    }

    Vector3 PPcontroller_acceleration(Vector3 position, Vector3 velocity, Vector3 forward, Vector3 right)
    {
        Vector3 sum_acceleration = new Vector3(0, 0, 0);
        Vector3 _tmp;
        //float bp_ratio=10F;

        _tmp = this.control3.desired_acceleration(position, velocity, right, forward);
        if (!(this.control3.is_lookahead_blocked(position)))
        {
            sum_acceleration += _tmp;
        }


        _tmp = this.control_backup.desired_acceleration(position, velocity, right, forward);


        if (this.control3.is_lookahead_blocked(position))
        {
            sum_acceleration = _tmp;
        }

        return sum_acceleration;
    }

    Vector3 Ray_acceleration(Vector3 position, Vector3 velocity, Vector3 forward, Vector3 right, bool normal)
    {
        Vector3 sum_acceleration = new Vector3(0, 0, 0);
        Vector3 _direction;
        Vector3 current_acc;
        float _dist;


        if (normal)
        {

            foreach (RayController ray in this.rays)
            {

                _direction = new Vector3(ray.relative_direction.z * forward.x + ray.relative_direction.x * forward.z,
                                        0,
                                        ray.relative_direction.z * forward.z - ray.relative_direction.x * forward.x);

                if (Vector3.Dot(forward, velocity) * Vector3.Dot(Vector3.forward, ray.relative_direction) < 0F)
                { // non driving direction
                    continue;
                }

                _dist = ray.max_distance;
                ray.max_distance = Mathf.Max((velocity.magnitude) * (velocity.magnitude) / (8F), _dist);

                if (Math.Abs(Vector3.Dot(Vector3.forward, ray.relative_direction)) < 0.99F)
                { // sideways - only steering
                    current_acc = ray.desired_acceleration(position, _direction);
                }
                else
                {
                    current_acc = ray.desired_acceleration(position, _direction);
                }
                ray.max_distance = _dist;

                sum_acceleration = sum_acceleration + current_acc;
            }
            return sum_acceleration;
        }
        else
        { // just for untrap mode
            foreach (RayController ray in this.rays)
            {

                // only consider rays that point to the same direction as the car is riding

                _direction = new Vector3(ray.relative_direction.z * forward.x + ray.relative_direction.x * forward.z,
                        0,
                        ray.relative_direction.z * forward.z - ray.relative_direction.x * forward.x);

                if (Math.Abs(Vector3.Dot(Vector3.forward, ray.relative_direction)) < -0.001F * this.trap_forward)
                { // alternating forward/backward
                    continue;
                }
                else
                {

                    current_acc = ray.desired_acceleration(position, _direction);

                }
                sum_acceleration = sum_acceleration + current_acc;
            }
            //Debug.Log(sum_acceleration);
            return sum_acceleration * 3F; // to speed up stuff
        }
    }
}

public class RayController
{
    public Vector3 relative_direction;
    public float max_distance, k_r;



    public RayController(Vector3 relative_direction, float max_distance, float k_r)
    {


        this.relative_direction = relative_direction;
        this.max_distance = max_distance;
        this.k_r = k_r;
    }

    public Vector3 desired_acceleration(Vector3 position, Vector3 direction) // the direction is supposed to be a TRANSFORMED version of this.relative direction!
    {
        // heavily based on CarAI script

        RaycastHit hit;
        Vector3 acc = new Vector3(0F, 0F, 0F);

        if (Physics.Raycast(position, direction, out hit, this.max_distance))
        {
            acc = direction * (this.max_distance - hit.distance) * (-1F) * this.k_r * (0.1F + 10F * (float)Math.Pow(1 - (hit.distance - 2F) / this.max_distance, 3F));
        }

        return acc;
    }

    public Vector3 desired_acceleration_drone(Vector3 position, Vector3 direction) // the direction is supposed to be a TRANSFORMED version of this.relative direction!
    {
        // heavily based on CarAI script

        RaycastHit hit;
        Vector3 acc = new Vector3(0F, 0F, 0F);

        if (Physics.Raycast(position, direction, out hit, this.max_distance))
        {
            acc = direction * (this.max_distance - hit.distance) * (-0.5F) * this.k_r * (0.1F + 10F * (float)Math.Pow(1 - hit.distance / this.max_distance, 2F) + 10F * (float)Math.Pow(1 - (hit.distance - 2F) / this.max_distance, 3F));
        }

        return acc;
    }
}