using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class PP_controller
{
    public polygon_path path;
    public float lookahead, max_deviation, k_p, k_d, v, mu, padding, vmax;

    public PP_controller(polygon_path _path, float _lookahead, float padding, float coarseness, float max_deviation, float k_p, float k_d, float v, float mu, float vmax = 1000f) // costructor that also does the preprocessing on the path

    // 1) linearly smooth path
    // 2) determine to what extent to increase resoultion - the max distance will roughly be the coarseness*lookahead
    // 3) after init, the update member function gives driing commands upon entering current position


    {
        this.lookahead = _lookahead;
        this.max_deviation = max_deviation;
        this.path = _path;
        this.k_p = k_p;
        this.v = v;
        this.k_d = k_d;
        this.mu = mu;
        this.padding = padding;
        this.vmax = vmax;

        // linear smoothing

        this.path = this.path.linear_smoothing(this.max_deviation); // ASSUMED THAT 

        // set speed limit

        this.path.set_speed_limit(this.mu);

        // hi-res generation for the tracker

        float resolution_goal = this.path.wp.max_segment_length() / coarseness / _lookahead; // note: no units 
        int current_resolution = 1;

        while ((float)current_resolution < resolution_goal)
        {
            this.path.double_resolution();
            current_resolution = current_resolution * 2;
        }

        // setting speed limit

        this.path.smooth_speed_limit(this.mu);


    }

    public Vector3 desired_acceleration(Vector3 position, Vector3 velocity, Vector3 right, Vector3 forward) // PD tracking of lookahead and keeping constant velocity, heavy copying from original (lecture) script
    {
        Vector3 lookahead_postion = this.path.lookahead_3d(position, this.lookahead);
        Vector3 position_error = lookahead_postion - position;

        Vector3 velocity_error = this.v * this.path.desired_driving_direction_3d(position) - velocity;

        // PD tracker, copied 

        Vector3 desired_acceleration = this.k_p * position_error + this.k_d * velocity_error;

        Vector3 steering = Vector3.Dot(desired_acceleration, right) * right;
        Vector3 acceleration = Vector3.Dot(desired_acceleration, forward) * forward;



        if (velocity.magnitude > Mathf.Min(this.path.find_speed_limit3d(position, this.lookahead), this.vmax))
        {
            if (Vector3.Dot(velocity, forward) > 0F)
            { // going forward, needs brake
                acceleration = -1F * this.k_d * (Mathf.Min(this.path.find_speed_limit3d(position, this.lookahead), this.vmax)) * forward;
            }
            else
            { // going backward, needs gas
                acceleration = this.k_d * (Mathf.Min(this.path.find_speed_limit3d(position, this.lookahead), this.vmax)) * forward;
            }

        }

        return steering + acceleration;


    }


    public Vector3 desired_acceleration_vacuum(Vector3 position, Vector3 velocity, Vector3 right, Vector3 forward) // PD tracking of lookahead and keeping constant velocity, heavy copying from original (lecture) script
    {
        Vector3 lookahead_postion = this.path.to_visit;
        Vector3 position_error = lookahead_postion - position;

        Vector3 velocity_error = this.v * this.path.desired_driving_direction_3d(position) - velocity;

        // PD tracker, copied 

        Vector3 desired_acceleration = this.k_p * position_error + this.k_d * velocity_error;

        Vector3 steering = Vector3.Dot(desired_acceleration, right) * right;
        Vector3 acceleration = Vector3.Dot(desired_acceleration, forward) * forward;



        if (velocity.magnitude > this.path.find_speed_limit3d(position, this.lookahead))
        {
            if (Vector3.Dot(velocity, forward) > 0F)
            { // going forward, needs brake
                acceleration = -1F * this.k_d * (velocity.magnitude - this.path.find_speed_limit3d(position, this.lookahead)) * forward;
            }
            else
            { // going backward, needs gas
                acceleration = this.k_d * (velocity.magnitude - this.path.find_speed_limit3d(position, this.lookahead)) * forward;
            }

        }

        return steering + acceleration;


    }

    public bool is_lookahead_blocked(Vector3 position)
    {

        Vector3 lookahead_postion = this.path.lookahead_3d(position, this.lookahead);
        Vector2 ahead = new Vector2(lookahead_postion.x, lookahead_postion.z);
        Vector2 pos = new Vector2(position.x, position.z);

        Line l = new Line(pos, ahead);
        EmbeddedLine el = new EmbeddedLine(l, this.padding, this.path._terrain_filename);

        return el.intersects_non_transversable();
    }

    public Vector3 back_to_path_acceleration(Vector3 position, float ratio)
    {
        Vector3 closest_postion = this.path.find_closest_point3d(position);
        Vector3 position_error = closest_postion - position;

        Vector3 desired_acceleration = this.k_p * ratio * position_error;

        return desired_acceleration;
    }

    public List<float> acceleration_to_command_normal_car(Vector3 desired_acceleration, Vector3 velocity, Vector3 position, Vector3 right, Vector3 forward)
    {
        float steering = Vector3.Dot(desired_acceleration, right);
        float acceleration = Vector3.Dot(desired_acceleration, forward);

        float[] commands = { steering, acceleration, acceleration, 0f };

        return new List<float>(commands);
    }

    public List<float> acceleration_to_command_reorient_car(Vector3 desired_acceleration, Vector3 velocity, Vector3 position, Vector3 right, Vector3 forward)
    {
        float steering = Vector3.Dot(desired_acceleration, right);
        float acceleration = Vector3.Dot(desired_acceleration, forward);


        float[] commands = { steering, acceleration, acceleration, 0f };

        return new List<float>(commands);
    }
    public List<float> acceleration_to_command_drone(Vector3 desired_acceleration, Vector3 right, Vector3 forward)
    {
        float horizontal = Vector3.Dot(desired_acceleration, right);
        float vertical = Vector3.Dot(desired_acceleration, forward);

        float[] commands = { horizontal, vertical };

        return new List<float>(commands);
    }

}
