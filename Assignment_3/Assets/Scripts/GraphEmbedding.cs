using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Newtonsoft.Json;

public class GraphEmbedding_versatile : GraphEmbedding
{

    public Dictionary<Tuple<int, int>, List<int>> visibile_turrets;
    public Dictionary<int, Vector3> turret_positions;
    public TerrainInfo terrain_info;
    //public Dictionary<int,Vector3> t;

    public GraphEmbedding_versatile(string terrain_filename, int resolution, float padding, Dictionary<int, Vector3> _turrets) :
    base(terrain_filename, resolution, padding)
    {

        this.terrain_info = this.get_terrain_data();
        this.turret_positions = _turrets;

        this.visibile_turrets = new Dictionary<Tuple<int, int>, List<int>>();

        foreach (var node in this.embedding.Keys)
        {
            this.visibile_turrets.Add(node, new List<int>());
        }

        foreach (int t in _turrets.Keys)
        {

            double x = _turrets[t].x;
            double z = _turrets[t].z;

            this.add_turret(t, x, z);
        }

    }

    // this.path_planner=new PathPlanner();
    // path=this.path_planner.plan_path(a,b,this.embedding,this.terrain_info,1);

    public List<Vector3> find_admissable_path(Vector3 start, Vector3 end)
    {
        PathPlanner planner = new PathPlanner();
        var path = planner.plan_path(start, end, this, this.terrain_info, 1);

        return path;
    }

    public List<Tuple<int, int>> st(Tuple<int, int> root)
    {

        List<Tuple<int, int>> visited_nodes = new List<Tuple<int, int>>();
        List<Tuple<int, int>> considered_nodes = new List<Tuple<int, int>>();
        SortedList<Tuple<int, int>, float> openset = new SortedList<Tuple<int, int>, float>();
        Tuple<int, int> current_node;

        openset.Add(root, (float)this.visibile_turrets[root].Count);
        visited_nodes.Add(root);
        considered_nodes.Add(root);

        bool same_guard_turrets;


        while (openset.Count > 0)
        {
            current_node = openset.Keys[0];
            openset.Remove(current_node);

            var adjacent_nodes = this.embedding[current_node];

            //Debug.Log("before: "+this.embedding[current_node].Count);

            // remove visited
            //adjacent_nodes.RemoveAll(l => considered_nodes.Contains(l)); - FUCKS UP EMBEDDING!

            //Debug.Log("after: "+this.embedding[current_node].Count);

            foreach (Tuple<int, int> node_ in adjacent_nodes)
            {

                if (considered_nodes.Contains(node_))
                {
                    continue;
                }

                considered_nodes.Add(node_);

                same_guard_turrets = true;


                if (this.visibile_turrets[root].Count != this.visibile_turrets[node_].Count)
                {
                    same_guard_turrets = false;
                }
                else
                {
                    foreach (int tu in this.visibile_turrets[node_])
                    {
                        if (!(this.visibile_turrets[root].Contains(tu)))
                        {
                            same_guard_turrets = false;
                        }
                    }
                }

                // if(this.visibile_turrets[node_].Count==max_turret && !(visited_nodes.Contains(node_))){
                if (same_guard_turrets && !(visited_nodes.Contains(node_)))
                {
                    openset.Add(node_, this.visibile_turrets[node_].Count);
                    visited_nodes.Add(node_);
                }
            }

        }

        return visited_nodes;

    }





    /*   public List<Tuple<int, int>> st(Tuple<int,int> root, int max_turret){

          List<Tuple<int, int>> visited_nodes = new List<Tuple<int, int>>();
          SortedList<Tuple<int,int>,float> openset=new SortedList<Tuple<int,int>,float>();
          Tuple<int,int> current_node;

          openset.Add(root,(float)this.visibile_turrets[root].Count);
          visited_nodes.Add(root);


          while(openset.Count>0){
              current_node=openset.Keys[0];
              openset.Remove(current_node);


              var adjacent_nodes = this.embedding[current_node];

              // remove visited
              adjacent_nodes.RemoveAll(l => visited_nodes.Contains(l));

              foreach(Tuple<int,int> node_ in adjacent_nodes){

                  if(this.visibile_turrets[node_].Count<=max_turret && !(visited_nodes.Contains(node_))){

                     openset.Add(node_,this.visibile_turrets[node_].Count);
                     visited_nodes.Add(node_);
                  }
              }

          }

          return visited_nodes;

      } */

    public float location_score(Tuple<int, int> node)
    { // avg seeing distance to visible turrets

        float x = this.get_x_pos(node.Item1, this.terrain_info);
        float y = this.get_z_pos(node.Item2, this.terrain_info);

        int n = this.visibile_turrets[node].Count;

        if (n == 0)
        {
            return 1000000f;
        }

        float dist = 0f;

        foreach (int t in this.visibile_turrets[node])
        {
            dist += (new Vector3(x, this.turret_positions[t].y, y) - this.turret_positions[t]).magnitude;
        }

        return dist / (float)n;


    }

    public Vector3 node_to_vec(Tuple<int, int> node)
    {
        float xx = this.get_x_pos(node.Item1, this.terrain_info);
        float zz = this.get_z_pos(node.Item2, this.terrain_info);


        // Debug.Log("xx: "+xx);
        // Debug.Log("zz: "+zz);

        return new Vector3(xx, 0f, zz);
    }

    public List<Tuple<int, int>> first_stage_nodes(Vector3 start)
    {
        int x = this.get_i_index(start.x, this.terrain_info);
        int y = this.get_j_index(start.z, this.terrain_info);

        Tuple<int, int> root = new Tuple<int, int>(x, y);

        var base_nodes = this.st(root);

        int min_neigh = 100;

        foreach (var node in base_nodes)
        {
            foreach (var neigh in this.embedding[node])
            {
                if (this.visibile_turrets[neigh].Count < min_neigh && (this.visibile_turrets[neigh].Count > 0))
                {
                    min_neigh = this.visibile_turrets[neigh].Count;
                }
            }
        }

        Debug.Log("min neigh: " + min_neigh);

        var uncovered_base_neighbors = new List<Tuple<int, int>>();
        foreach (var node in base_nodes)
        {
            foreach (var neigh_ in this.embedding[node])
            {
                if ((!(uncovered_base_neighbors.Contains(neigh_)) && (this.visibile_turrets[neigh_].Count <= min_neigh)))
                {
                    uncovered_base_neighbors.Add(neigh_);
                    Debug.Log("ubn add:" + neigh_ + ", count: " + this.visibile_turrets[neigh_].Count + " vs " + min_neigh + "    " + (this.visibile_turrets[neigh_].Count <= min_neigh));

                }
            }
        }

        List<Tuple<int, int>> second_stage_nodes = new List<Tuple<int, int>>();
        List<Tuple<int, int>> tmp = new List<Tuple<int, int>>();

        while (uncovered_base_neighbors.Count > 0)
        {
            tmp = this.st(uncovered_base_neighbors[0]);
            uncovered_base_neighbors.RemoveAll(l => tmp.Contains(l));
            tmp.RemoveAll(l => second_stage_nodes.Contains(l));

            second_stage_nodes.AddRange(tmp);
        }

        base_nodes.AddRange(second_stage_nodes);

        return base_nodes;
    }

    public Tuple<int, int> optimal_position(List<Tuple<int, int>> nodes)
    {
        float optimal_score = 1000000f;
        Tuple<int, int> optimal_position = new Tuple<int, int>(-1, -1);

        foreach (var node in nodes)
        {
            if (this.location_score(node) < optimal_score)
            {
                optimal_position = node;
                optimal_score = this.location_score(node);
            }
        }

        List<Tuple<int, int>> patch = new List<Tuple<int, int>>();

        return optimal_position;
    }

    /* public Tuple<int,int> find_1stage_st(Vector3 start){ // needed for p5 specifically

        int x=this.get_i_index(start.x,this.terrain_info);
        int y=this.get_j_index(start.z,this.terrain_info);

        Tuple<int,int> root=new Tuple<int,int>(x,y);

        var base_nodes=this.st(root,this.visibile_turrets[root].Count);

        int min_neigh=100;

        foreach(var node in base_nodes){
            foreach(var neigh in this.embedding[node]){
                if((this.visibile_turrets[neigh].Count<min_neigh) && (this.visibile_turrets[neigh].Count>0)){
                    min_neigh=this.visibile_turrets[neigh].Count;
                }
            }
        }

        var stage_1_nodes=this.st(root,min_neigh);

        float optimal_score=1000000f;
        Tuple<int,int> optimal_position=new Tuple<int,int>(-1,-1);

        foreach(var node in stage_1_nodes){
            if(this.location_score(node)<optimal_score){
                optimal_position=node;
                optimal_score=this.location_score(node);
            }
        }

        return optimal_position;

    } */

    public override double node_distance(Tuple<int, int> node1, Tuple<int, int> node2)
    {

        double base_ = Math.Sqrt(((node1.Item1 - node2.Item1) * (node1.Item1 - node2.Item1) +
                    (node1.Item2 - node2.Item2) * (node1.Item2 - node2.Item2)));

        double sum_vis = (this.visibile_turrets[node1].Count + this.visibile_turrets[node2].Count) + 0.01;

        return base_ * sum_vis * 100000; // force algs to go over safe terrain the most it is possible

    }

    public bool is_traversable_coord_no_pad(double x1, double y1, double x2, double y2)
    {

        Vector2 start = new Vector2((float)x1, (float)y1);
        Vector2 end = new Vector2((float)x2, (float)y2);

        EmbeddedLine el = new EmbeddedLine(new Line(start, end), 0f, this._terrain_filename);

        return !(el.intersects_non_transversable());
    }

    public void delete_turret(int t)
    {
        foreach (var node in this.visibile_turrets.Keys)
        {
            if (this.visibile_turrets[node].Contains(t))
            {

                int c = this.visibile_turrets[node].Count;
                this.visibile_turrets[node].Remove(t);

                /*   if((this.visibile_turrets[node].Count-c)!=-1){
                      Debug.Log(node+ " node: "+ (this.visibile_turrets[node].Count-c));
                  } */
                /*    else{
                       float xx=this.get_x_pos(node.Item1,this.terrain_info);
                       float yy=this.get_z_pos(node.Item2,this.terrain_info);
                       Debug.DrawLine(new Vector3(350f,0f,95f),new Vector3(xx,0f,yy),Color.white,3f);
                   } */
            }
        }
    }

    public void add_turret(int t, double x, double y)
    { // t is supposed to be the index
        foreach (var node in this.visibile_turrets.Keys)
        {


            float xx = this.get_x_pos(node.Item1, this.terrain_info);
            float yy = this.get_z_pos(node.Item2, this.terrain_info);

            if (!this.visibile_turrets[node].Contains(t) && this.is_traversable_coord_no_pad(x, y, xx, yy))
            {
                this.visibile_turrets[node].Add(t);
            }
        }
    }

}

public class GraphEmbedding
// class representing a graph embedding used in the code
{
    // ----------------------------- attributes -----------------------------

    // string containing the path to the terrain
    public string _terrain_filename;

    // dictionary attribute containing the embedding
    public Dictionary<Tuple<int, int>, List<Tuple<int, int>>> embedding;

    // terrain storied in 2D float array
    public int[,] terrain_arr;

    // indices of the terrain cells stored in 2D array with coordinates 
    public int[,,] terrain_indices;

    // int representing the resolution of the embedding, !!!must be power of 2!!!
    // for example, 1 will give embedding where each terrain block represents a node
    // resolution=4 will mean that each building block is divided into 4 equal sized blocks
    // so in total there will be 4 times more nodes in the embedding
    public int resolution;

    // weights of the nodes in the embedding
    public Dictionary<Tuple<int, int>, double> weights;

    // padding to control how close the nodes can be to obstacles
    public float padding;


    // -------------------------- methods ---------------------------

    // constructor
    public GraphEmbedding(string terrain_filename, int resolution, float padding)
    {
        this._terrain_filename = terrain_filename;
        this.padding = padding;
        this.resolution = resolution;
        this.terrain_arr = this.read_terrain();
        this.terrain_indices = this.get_terrain_indices();
        this.set_embedding_from_terrain();
        this.init_weights();
        // note: change if necessary

    }

    public void draw_embedding(TerrainInfo terrain_info)
    {
        foreach (var k in this.embedding.Keys)
        {
            var adjacent = this.embedding[k];
            foreach (var adj in adjacent)
            {
                Debug.DrawLine(
                    new Vector3(this.get_x_pos(k.Item1, terrain_info), 0, this.get_z_pos(k.Item2, terrain_info)),
                    new Vector3(this.get_x_pos(adj.Item1, terrain_info), 0, this.get_z_pos(adj.Item2, terrain_info)),
                    Color.blue, 100f
                );
                Debug.DrawLine(
                    new Vector3(this.get_x_pos(k.Item1, terrain_info), 0, this.get_z_pos(k.Item2, terrain_info)),
                    new Vector3(this.get_x_pos(k.Item1, terrain_info) + 1f, 0, this.get_z_pos(k.Item2, terrain_info) + 1f),
                    Color.black, 100f
                );
            }
        }
    }

    public TerrainInfo get_terrain_data()
    {
        var jsonTextFile = Resources.Load<TextAsset>(this._terrain_filename);
        return TerrainInfo.CreateFromJSON(jsonTextFile.text);
    }

    // public method that reads the contents of terrain filename (string)
    public int[,] read_terrain()
    {

        // load the traversability matrix
        var terrain_json = this.get_terrain_data().traversability;

        var shape_x = terrain_json.GetLength(0);
        var shape_y = terrain_json.GetLength(1);

        var new_shape_x = shape_x * this.resolution;
        var new_shape_y = shape_y * this.resolution;

        // apply resolution (expand the terrain array)
        // possible TODO: make it dynamic given environment and obstacles
        // possible TODO: don't apply it on obstacles
        var new_terrain = new int[new_shape_x, new_shape_y];

        for (int col_idx = 0; col_idx < shape_x; col_idx++)
        {
            for (int row_idx = 0; row_idx < shape_y; row_idx++)
            {
                for (int i_idx = 0; i_idx < this.resolution; i_idx++)
                {
                    for (int j_idx = 0; j_idx < this.resolution; j_idx++)
                    {
                        new_terrain[col_idx * this.resolution + i_idx, row_idx * this.resolution + j_idx] = (int)terrain_json[col_idx, row_idx];
                    }
                }
            }
        }

        return new_terrain;
    }

    public void set_weights(Tuple<int, int> node, List<Tuple<int, int>> visited_nodes, int distance)
    {

        var adjacent_nodes = new List<Tuple<int, int>>(this.embedding[node]);
        adjacent_nodes.RemoveAll(l => visited_nodes.Contains(l));

        if (adjacent_nodes.Count == 0)
        {
            return;
        }

        foreach (var adjacent_node in adjacent_nodes)
        {
            this.weights[adjacent_node] = distance;
            visited_nodes.Add(adjacent_node);
            this.set_weights(adjacent_node, visited_nodes, ++distance);
        }

    }

    public void init_weights()
    {

        // initialize weights to random between 0 and 1

        System.Random rng = new System.Random();
        Dictionary<Tuple<int, int>, double> weights = new Dictionary<Tuple<int, int>, double>();
        foreach (var node in this.embedding.Keys)
        {
            //weights.Add(node, rng.Next(0, 10000)/10000f);
            weights.Add(node, 0F);

        }
        this.weights = weights;
    }

    public void update_weights(List<Tuple<int, int>> nodes, Tuple<bool, List<double>> walks)
    {

        // update the weights
        var multiplier = 1;

        if (walks.Item1)
        {
            multiplier = 1;
        }
        else
        {
            // penalize the nodes that did not lead to the goal
            multiplier = 20;
        }

        if (walks.Item1)
        {

            for (int i = 0; i < nodes.Count; i++)
            {
                // if goal is reached, set the weight as the node distance to the goal of the given node
                this.weights[nodes[i]] = walks.Item2[i];
            }

        }
        else
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                // goal was not reached, multiply the current weight
                this.weights[nodes[i]] = this.weights[nodes[i]] * (i + 1);
            }
        }

    }

    // public void reset_except(List<Tuple<int, int>> nodes){
    //     for(int i=0; i<nodes.Count; i++){
    //         for(int j=0; j<this.weights.Count; j++){
    //             if (this.weights[i] != )
    //         }
    //     }

    // }

    // method that returns 2D array of indices of the nodes
    public int[,,] get_terrain_indices()
    {

        // generate the indices
        var indices = new int[this.terrain_arr.GetLength(0), this.terrain_arr.GetLength(1), 2];

        // each cell gets an individual index
        for (int col_idx = 0; col_idx < this.terrain_arr.GetLength(0); col_idx++)
        {
            for (int row_idx = 0; row_idx < this.terrain_arr.GetLength(1); row_idx++)
            {
                indices[col_idx, row_idx, 0] = col_idx;
                indices[col_idx, row_idx, 1] = row_idx;
            }
        }

        return indices;
    }



    public float get_x_pos(int i, TerrainInfo terrain_info)
    {
        float step = (terrain_info.x_high - terrain_info.x_low) / (terrain_info.x_N * this.resolution);
        return terrain_info.x_low + step / 2 + step * i;

    }


    public float get_z_pos(int j, TerrainInfo terrain_info)
    {
        float step = (terrain_info.z_high - terrain_info.z_low) / (terrain_info.z_N * this.resolution);
        return terrain_info.z_low + step / 2 + step * j;
    }


    public int get_i_index(float x, TerrainInfo terrain_info)
    {
        int index = (int)Mathf.Floor(terrain_info.x_N * this.resolution *
                    (x - terrain_info.x_low) / (terrain_info.x_high - terrain_info.x_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > terrain_info.x_N * this.resolution - 1)
        {
            index = terrain_info.x_N * this.resolution - 1;
        }
        return index;
    }


    public int get_j_index(float z, TerrainInfo terrain_info)
    {
        int index = (int)Mathf.Floor(terrain_info.z_N * this.resolution *
                         (z - terrain_info.z_low) / (terrain_info.z_high - terrain_info.z_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > terrain_info.z_N * this.resolution - 1)
        {
            index = terrain_info.z_N * this.resolution - 1;
        }
        return index;
    }

    public float[,,] get_terrain_coordinates()
    {

        // return the coordinates of the terrain blocks

        var terrain = this.terrain_arr;
        var shape_x = terrain.GetLength(0);
        var shape_y = terrain.GetLength(1);
        float[,,] terrain_coords = new float[shape_x, shape_y, 2];

        var terrain_info = this.get_terrain_data();

        for (int x = 0; x < shape_x; x++)
        {
            for (int y = 0; y < shape_y; y++)
            {
                // terrain_coords[x,y,0] = terrain_info.get_x_pos(this.terrain_indices[x,y,0]);
                // terrain_coords[x,y,1] = terrain_info.get_z_pos(this.terrain_indices[x,y,1]);
                terrain_coords[x, y, 0] = this.get_x_pos(this.terrain_indices[x, y, 0], terrain_info);
                terrain_coords[x, y, 1] = this.get_z_pos(this.terrain_indices[x, y, 1], terrain_info);

            }
        }

        return terrain_coords;
    }


    public Vector3[,] get_terrain_coordinates_vector3()
    {

        var terrain = this.terrain_arr;
        var shape_x = terrain.GetLength(0);
        var shape_y = terrain.GetLength(1);
        Vector3[,] terrain_coords = new Vector3[shape_x, shape_y];

        var terrain_info = this.get_terrain_data();
        for (int x = 0; x < shape_x; x++)
        {
            for (int y = 0; y < shape_y; y++)
            {
                terrain_coords[x, y] = new Vector3(this.get_x_pos(this.terrain_indices[x, y, 0], terrain_info),
                                                  0,
                                                  this.get_z_pos(this.terrain_indices[x, y, 1], terrain_info));

            }
        }

        return terrain_coords;

    }

    public virtual double node_distance(Tuple<int, int> node1, Tuple<int, int> node2)
    {

        // euclidean
        var distance = Math.Sqrt(((node1.Item1 - node2.Item1) * (node1.Item1 - node2.Item1) +
                    (node1.Item2 - node2.Item2) * (node1.Item2 - node2.Item2)));

        return distance;
    }

    public double euclidean_node_distance(Tuple<int, int> node1, Tuple<int, int> node2)
    {

        // euclidean
        var distance = Math.Sqrt(((node1.Item1 - node2.Item1) * (node1.Item1 - node2.Item1) +
                    (node1.Item2 - node2.Item2) * (node1.Item2 - node2.Item2)));

        return distance;
    }

    public double distance(double x1, double x2, double y1, double y2)
    {
        return Math.Sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    public double h(Tuple<int, int> target_node, Tuple<int, int> goal_node)
    { // REWRITTEN TO VANILLA NODE DISTANCE
        return this.weights[target_node] + this.euclidean_node_distance(target_node, goal_node) / 2;
        //return this.euclidean_node_distance(target_node, goal_node)*0.5;
    }



    public bool is_adjacent(int x1, int y1, int x2, int y2)
    {
        // method that checks if two cells are adjacent
        // note: we treat one point to be adjacent to itself
        //return ( Math.Abs(x1 - x2) <= 1 && Math.Abs(y1 - y2) <= 1);

        return (Math.Abs(x1 - x2) <= 1) && (Math.Abs(y1 - y2) <= 1);
    }

    public bool is_different(int x1, int y1, int x2, int y2)
    {
        //return !(x1 == x2 && y1 == y2);

        if (x1 == x2)
        {
            if (y1 == y2)
            {
                return false;
            }
        }

        return true;
    }

    /*  public bool is_traversable(int x1, int y1, int x2, int y2){
         // checks if we can traverse through two points, i.e. both are paths and not terrain blockades
         return (this.terrain_arr[x1, y1] == 0.0 && this.terrain_arr[x2, y2] == 0.0);
     } */

    public bool is_traversable(int x1, int y1, int x2, int y2)
    {
        // checks if we can traverse through two points, i.e. both are paths and not terrain blockades

        var terrain_info = this.get_terrain_data();
        Vector2 start = new Vector2(this.get_x_pos(x1, terrain_info), this.get_z_pos(y1, terrain_info));
        Vector2 end = new Vector2(this.get_x_pos(x2, terrain_info), this.get_z_pos(y2, terrain_info));

        EmbeddedLine el = new EmbeddedLine(new Line(start, end), this.padding, this._terrain_filename);

        return (this.terrain_arr[x1, y1] == 0.0) && (this.terrain_arr[x2, y2] == 0.0) && (!(el.intersects_non_transversable()));
        //return (this.terrain_arr[x1, y1] == 0) && (this.terrain_arr[x2, y2] == 0);

        //return true;

    }


    // method that accepts the array representing the terrain (2D float array)
    // and sets it as the object's in correct format
    public void set_embedding_from_terrain()
    {

        var indices = this.terrain_indices;
        var terrain_arr = this.terrain_arr;

        Dictionary<Tuple<int, int>, List<Tuple<int, int>>> embedding = new Dictionary<Tuple<int, int>, List<Tuple<int, int>>>();

        var shape_x = terrain_arr.GetLength(0);
        var shape_y = terrain_arr.GetLength(1);

        // for each index
        // possible TODO: optimize to traverse only through adjacent cells
        for (int x1 = 0; x1 < shape_x; x1++)
        {
            for (int y1 = 0; y1 < shape_y; y1++)
            {

                //var curr_x = indices[x1, y1, 0];
                //var curr_y = indices[x1, y1, 1];

                Tuple<int, int> curr_idx = new Tuple<int, int>(x1, y1);

                // check other indices
                for (int x2 = 0; x2 < shape_x; x2++)
                {
                    for (int y2 = 0; y2 < shape_y; y2++)
                    {
                        if (this.is_adjacent(x1, y1, x2, y2) && (this.is_traversable(x1, y1, x2, y2) && this.is_different(x1, y1, x2, y2)))
                        {
                            //if((x2-x1+y2-y1==1)){
                            // possible TODO: optimize not to check whether key exists every time

                            // check if cell idx already in embedding
                            if (embedding.ContainsKey(curr_idx))
                            {
                                // append to list
                                //Debug.Log('a');
                                embedding[curr_idx].Add(new Tuple<int, int>(x2, y2));
                            }
                            else
                            {
                                // add new index and value
                                //embedding.Add(curr_idx, new List<Tuple<int,int>>{ new Tuple<int,int>(indices[x2, y2,0], indices[x2, y2,1]) } );
                                embedding.Add(curr_idx, new List<Tuple<int, int>> { new Tuple<int, int>(x2, y2) });
                            }
                        }
                    }
                }


            }
        }

        // set the embedding as attribute of the object
        this.embedding = embedding;

    }



}

