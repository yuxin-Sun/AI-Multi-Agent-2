using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class TSP_graph
{

    public Vector3 depot;
    public List<Vector3> customers;
    public GraphEmbedding embedding;
    public PathPlanner path_planner;
    public TerrainInfo terrain_info;
    public float[,] d; // distance matrix
    public int n;

    public TSP_graph(Vector3 depot, List<Vector3> customers, GraphEmbedding embedding, TerrainInfo terrain_info)
    {
        this.depot = depot; // don't bother copying, no use
        this.customers = customers;
        this.embedding = embedding;
        this.terrain_info = terrain_info;
        this.path_planner = new PathPlanner();

        this.n = 1 + customers.Count;

        this.d = new float[n, n];

        for (int i = 0; i < n; i++)
        { // fill d with dummy values
            for (int j = 0; j < n; j++)
            {
                this.d[i, j] = -1F;
            }
        }

    }

    public void set_distance(int i, int j, bool draw)
    { // calculates shortest path distance with A*, or directly if points see one another. Idx 0 is the depot
        Vector3 a, b;
        if (i == 0) { a = this.depot; } else { a = this.customers[i - 1]; }
        if (j == 0) { b = this.depot; } else { b = this.customers[j - 1]; }
        float length = 0F;

        EmbeddedLine el = new EmbeddedLine(new Line(a, b), this.embedding.padding, this.embedding._terrain_filename);

        List<Vector3> path;

        if (el.intersects_non_transversable())
        {
            path = this.path_planner.plan_path(a, b, this.embedding, this.terrain_info, 1);
            for (int k = 0; k < path.Count - 1; k++)
            {
                length += (path[k + 1] - path[k]).magnitude;
            }
        }
        else
        {
            path = new List<Vector3>();
            path.Add(a);
            path.Add(b);
            length = (a - b).magnitude;
        }

        if (draw)
        {
            Vector3 _old_wp = path[0];
            foreach (var wp in path)
            {
                Debug.DrawLine(_old_wp, wp, Color.green, 100f);
                //Debug.Log("segment drawn");
                _old_wp = wp;
            }
        }

        this.d[i, j] = length;
        this.d[j, i] = length;

    }

    public void draw_edge(Tuple<int, int> edge)
    {

        int i = edge.Item1;
        int j = edge.Item2;

        Vector3 a, b;
        if (i == 0) { a = this.depot; } else { a = this.customers[i - 1]; }
        if (j == 0) { b = this.depot; } else { b = this.customers[j - 1]; }

        Debug.DrawLine(a, b, Color.red, 100f);

    }

    public Vector3 find_node(int i)
    {
        if (i == 0)
        {
            return this.depot;
        }
        else
        {
            return this.customers[i - 1];
        }
    }

    public void set_distance_matrix(bool draw)
    {
        for (int i = 0; i < this.n; i++)
        {
            for (int j = 0; j < i; j++)
            {
                this.set_distance(i, j, draw);
            }
        }
    }


    public float arc_length(List<int> nodes)
    {
        float distance_so_far = 0f;
        for (int k = 0; k < nodes.Count - 1; k++)
        {
            distance_so_far += this.d[nodes[k], nodes[k + 1]];
        }

        return distance_so_far;
    }

    public List<int> slice_list(List<int> list, int k, int l)
    {
        List<int> _list = new List<int>();

        for (int i = k; i <= l; i++)
        {
            _list.Add(list[i]);
        }

        return _list;
    }

    public List<int> christofides_naive()
    {

        List<int> all = new List<int>();
        int[] counter = new int[this.n];

        for (int i = 0; i < this.n; i++)
        {
            counter[i] = 0;
            all.Add(i);
        }


        List<Tuple<int, int>> edges = this.mst(all);
        List<int> odd_nodes = new List<int>();





        foreach (Tuple<int, int> edge in edges)
        {
            counter[edge.Item1] += 1;
            counter[edge.Item2] += 1;
        }

        for (int i = 0; i < this.n; i++)
        {
            if (counter[i] % 2 == 1)
            {
                odd_nodes.Add(i);
            }
        }

        Tuple<int, int> _edge;

        while (odd_nodes.Count > 0)
        {
            _edge = find_shortest_edge(odd_nodes);
            odd_nodes.Remove(_edge.Item1);
            odd_nodes.Remove(_edge.Item2);

            edges.Add(_edge);

        }

        List<int> euler = euler_tour(0, edges);

        List<int> hamilton = new List<int>();

        foreach (int node in euler)
        {
            if (!hamilton.Contains(node))
            {
                hamilton.Add(node);
            }
        }

        return hamilton;



    }

    public List<int> euler_tour(int u, List<Tuple<int, int>> edges)
    { // will destroy edge set!

        List<int> _tour = new List<int>();

        List<Tuple<int, int>> edge_copy = new List<Tuple<int, int>>();
        foreach (var edge in edges)
        {
            edge_copy.Add(new Tuple<int, int>(edge.Item1, edge.Item2));
        }


        foreach (Tuple<int, int> edge in edges)
        {


            if (u == edge.Item1)
            {
                edge_copy.Remove(edge);
                foreach (int __node in euler_tour(edge.Item2, edge_copy))
                {
                    _tour.Add(__node);
                }
            }
            if (u == edge.Item2)
            {
                edge_copy.Remove(edge);
                foreach (int __node in euler_tour(edge.Item1, edge_copy))
                {
                    _tour.Add(__node);
                }

            }
        }

        _tour.Add(u);

        return _tour;

    }

    public Tuple<int, int> find_shortest_edge(List<int> used_nodes)
    {

        Tuple<int, int> edge = new Tuple<int, int>(-1, -1);

        int k = used_nodes.Count;
        float min_ = 1000000F;

        for (int i = 0; i < k; i++)
        {
            for (int j = 0; j < i; j++)
            {
                if (this.d[used_nodes[i], used_nodes[j]] < min_)
                {
                    min_ = this.d[used_nodes[i], used_nodes[j]];
                    edge = new Tuple<int, int>(used_nodes[i], used_nodes[j]);
                }
            }

        }
        return edge;

    }

    public List<Tuple<int, int>> mst(List<int> used_nodes)
    { // compute minimal spanning tree, skip nodes 

        // since our graph is complete, we have a very special case of Prim

        List<int> visited_nodes = new List<int>();
        List<int> unvisited_nodes = new List<int>();
        List<Tuple<int, int>> edges = new List<Tuple<int, int>>();

        visited_nodes.Add(used_nodes[0]);
        for (int i = 1; i < used_nodes.Count; i++)
        {
            unvisited_nodes.Add(used_nodes[i]);
        }


        float min_length;
        int start, end;
        start = -1;
        end = -1;

        while (unvisited_nodes.Count > 0)
        {
            min_length = 100000F;
            foreach (int _start in visited_nodes)
            {
                foreach (int _end in unvisited_nodes)
                {
                    if (this.d[_start, _end] < min_length)
                    {
                        min_length = this.d[_start, _end];
                        start = _start;
                        end = _end;
                    }
                }
            }

            visited_nodes.Add(end);
            unvisited_nodes.Remove(end);

            edges.Add(new Tuple<int, int>(start, end));
        }

        return edges;
    }

}

public class PathPlanner
{

    public List<Vector3> plan_path(Vector3 start_pos, Vector3 goal_pos, GraphEmbedding_versatile embedding, TerrainInfo terrain_info, int train_iters)
    {

        // create empty path
        List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> node_path = new List<Tuple<int, int>>();

        var coords = embedding.get_terrain_coordinates_vector3();

        var best_distance = 999999999.0;
        var best_path = new List<Vector3>();
        var best_walks = 99999999;


        for (int i = 0; i < train_iters; i++)
        {
            //var path_results = greedy_path(start_pos, goal_pos, embedding, terrain_info);
            //var path_results = bfs(start_pos, goal_pos, embedding, terrain_info);
            var path_results = a_star(start_pos, goal_pos, embedding, terrain_info);
            //******************************
            var reached_goal = path_results.Item2;
            Debug.Log(reached_goal.ToString());
            var paths = path_results.Item1;
            if (paths.Item1.Count == 0 || paths.Item2.Count == 0)
            {
                continue;
            }
            path = paths.Item1;
            node_path = paths.Item2;

            if (this.get_path_distance(embedding, path) < best_distance && path.Count < best_walks && reached_goal)
            {
                best_distance = this.get_path_distance(embedding, path);
                best_walks = path.Count;
                best_path = path;
            }

            // embedding.update_weights(node_path, this.get_path_walks(embedding, path, reached_goal));
        }

        return best_path;
    }



    public List<Vector3> plan_path(Vector3 start_pos, Vector3 goal_pos, GraphEmbedding embedding, TerrainInfo terrain_info, int train_iters)
    {

        // create empty path
        List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> node_path = new List<Tuple<int, int>>();

        var coords = embedding.get_terrain_coordinates_vector3();

        var best_distance = 999999999.0;
        var best_path = new List<Vector3>();
        var best_walks = 99999999;


        for (int i = 0; i < train_iters; i++)
        {
            //var path_results = greedy_path(start_pos, goal_pos, embedding, terrain_info);
            //var path_results = bfs(start_pos, goal_pos, embedding, terrain_info);
            var path_results = a_star(start_pos, goal_pos, embedding, terrain_info);
            //******************************
            var reached_goal = path_results.Item2;
            Debug.Log(reached_goal.ToString());
            var paths = path_results.Item1;
            if (paths.Item1.Count == 0 || paths.Item2.Count == 0)
            {
                continue;
            }
            path = paths.Item1;
            node_path = paths.Item2;

            if (this.get_path_distance(embedding, path) < best_distance && path.Count < best_walks && reached_goal)
            {
                best_distance = this.get_path_distance(embedding, path);
                best_walks = path.Count;
                best_path = path;
            }

            // embedding.update_weights(node_path, this.get_path_walks(embedding, path, reached_goal));
        }

        return best_path;
    }

    private Tuple<bool, List<double>> get_path_walks(GraphEmbedding emb, List<Vector3> path, bool reached_goal)
    {

        var walks = new List<double>();

        for (int i = 0; i < path.Count; i++)
        {
            walks.Add(path.Count - i);
        }

        return new Tuple<bool, List<double>>(reached_goal, walks);
    }

    private double get_path_distance(GraphEmbedding emb, List<Vector3> path)
    {

        double distance = 0.0;
        var start_node = path[0];
        foreach (var cord in path)
        {
            distance = distance + emb.distance(start_node[0], cord[0], start_node[2], cord[2]);
            start_node = cord;
        }
        return distance;
    }

    private static Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool> a_star(Vector3 start_pos, Vector3 goal_pos, GraphEmbedding_versatile embedding, TerrainInfo terrain_info)
    {
        Tuple<int, int> start_node = new Tuple<int, int>(embedding.get_i_index(start_pos[0], terrain_info),
                                                       embedding.get_j_index(start_pos[2], terrain_info)
                                                       );
        Tuple<int, int> goal_node = new Tuple<int, int>(embedding.get_i_index(goal_pos[0], terrain_info),
                                                      embedding.get_j_index(goal_pos[2], terrain_info)
                                                      );

        //List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> visited_nodes = new List<Tuple<int, int>>();

        SortedList<Tuple<int, int>, float> openset = new SortedList<Tuple<int, int>, float>();

        List<Tuple<int, int>> to_visit = new List<Tuple<int, int>>();

        Tuple<int, int> current_node;
        Dictionary<Tuple<int, int>, Tuple<int, int>> parent = new Dictionary<Tuple<int, int>, Tuple<int, int>>();
        Dictionary<Tuple<int, int>, float> g = new Dictionary<Tuple<int, int>, float>();
        Dictionary<Tuple<int, int>, bool> in_queue = new Dictionary<Tuple<int, int>, bool>();

        List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> node_path = new List<Tuple<int, int>>();

        //path.Add(start_pos);

        openset.Add(start_node, (float)embedding.h(start_node, goal_node));
        in_queue.Add(start_node, true);
        g.Add(start_node, 0F);
        parent.Add(start_node, new Tuple<int, int>(-1, -1)); // root node of path

        //visited_nodes.Add(start_node);


        /*   foreach(Tuple<int,int> node in embedding.embedding[start_node]){
              to_visit.Add(node);
              parent.Add(node,start_node);
              visited_nodes.Add(node);

          } */

        float tentative_g;

        while (openset.Count > 0)
        {
            current_node = openset.Keys[0];
            openset.Remove(current_node);
            in_queue[current_node] = false;

            if (current_node == goal_node)
            {
                break; // and reconstruct path
            }

            var adjacent_nodes = embedding.embedding[current_node];

            // remove visited
            //adjacent_nodes.RemoveAll(l => visited_nodes.Contains(l));

            foreach (Tuple<int, int> node_ in adjacent_nodes)
            {
                tentative_g = g[current_node] + (float)embedding.node_distance(current_node, node_);

                if (!(g.ContainsKey(node_)) || (tentative_g < g[node_]))
                {

                    if (parent.ContainsKey(node_))
                    {
                        parent[node_] = current_node;
                    }
                    else { parent.Add(node_, current_node); }

                    if (g.ContainsKey(node_))
                    {
                        g[node_] = tentative_g;
                    }
                    else { g.Add(node_, tentative_g); }

                    if (!(in_queue.ContainsKey(node_)) || in_queue[node_] == false)
                    {
                        if (in_queue.ContainsKey(node_))
                        {
                            in_queue[node_] = true;
                            openset.Add(node_, g[node_] + (float)embedding.h(node_, goal_node));
                        }
                        else
                        {
                            in_queue.Add(node_, true);
                            openset.Add(node_, g[node_] + (float)embedding.h(node_, goal_node));
                        }
                    }
                }
            }

        }

        if (parent.ContainsKey(goal_node))
        {
            current_node = goal_node;
            node_path.Add(current_node);
            path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));
            while (parent[current_node].Item1 != -1 || parent[current_node].Item2 != -1)
            {
                current_node = parent[current_node];
                node_path.Add(current_node);
                path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));

            }

            // start node - not to leave it out

            // node_path.Add(current_node); 
            // path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));

            path.Reverse();
            node_path.Reverse();
            var paths_finished = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
            return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_finished, true);
        }
        else
        {
            Debug.Log("Path not found, no adjacent nodes.");
            var paths_no_adj = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
            return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_no_adj, false);
        }

    }



    private static Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool> a_star(Vector3 start_pos, Vector3 goal_pos, GraphEmbedding embedding, TerrainInfo terrain_info)
    {
        Tuple<int, int> start_node = new Tuple<int, int>(embedding.get_i_index(start_pos[0], terrain_info),
                                                       embedding.get_j_index(start_pos[2], terrain_info)
                                                       );
        Tuple<int, int> goal_node = new Tuple<int, int>(embedding.get_i_index(goal_pos[0], terrain_info),
                                                      embedding.get_j_index(goal_pos[2], terrain_info)
                                                      );

        //List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> visited_nodes = new List<Tuple<int, int>>();

        SortedList<Tuple<int, int>, float> openset = new SortedList<Tuple<int, int>, float>();

        List<Tuple<int, int>> to_visit = new List<Tuple<int, int>>();

        Tuple<int, int> current_node;
        Dictionary<Tuple<int, int>, Tuple<int, int>> parent = new Dictionary<Tuple<int, int>, Tuple<int, int>>();
        Dictionary<Tuple<int, int>, float> g = new Dictionary<Tuple<int, int>, float>();
        Dictionary<Tuple<int, int>, bool> in_queue = new Dictionary<Tuple<int, int>, bool>();

        List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> node_path = new List<Tuple<int, int>>();

        //path.Add(start_pos);

        openset.Add(start_node, (float)embedding.h(start_node, goal_node));
        in_queue.Add(start_node, true);
        g.Add(start_node, 0F);
        parent.Add(start_node, new Tuple<int, int>(-1, -1)); // root node of path

        //visited_nodes.Add(start_node);


        /*   foreach(Tuple<int,int> node in embedding.embedding[start_node]){
              to_visit.Add(node);
              parent.Add(node,start_node);
              visited_nodes.Add(node);

          } */

        float tentative_g;

        while (openset.Count > 0)
        {
            current_node = openset.Keys[0];
            openset.Remove(current_node);
            in_queue[current_node] = false;

            if (current_node == goal_node)
            {
                break; // and reconstruct path
            }

            var adjacent_nodes = embedding.embedding[current_node];

            // remove visited
            //adjacent_nodes.RemoveAll(l => visited_nodes.Contains(l));

            foreach (Tuple<int, int> node_ in adjacent_nodes)
            {
                tentative_g = g[current_node] + (float)embedding.node_distance(current_node, node_);

                if (!(g.ContainsKey(node_)) || (tentative_g < g[node_]))
                {

                    if (parent.ContainsKey(node_))
                    {
                        parent[node_] = current_node;
                    }
                    else { parent.Add(node_, current_node); }

                    if (g.ContainsKey(node_))
                    {
                        g[node_] = tentative_g;
                    }
                    else { g.Add(node_, tentative_g); }

                    if (!(in_queue.ContainsKey(node_)) || in_queue[node_] == false)
                    {
                        if (in_queue.ContainsKey(node_))
                        {
                            in_queue[node_] = true;
                            openset.Add(node_, g[node_] + (float)embedding.h(node_, goal_node));
                        }
                        else
                        {
                            in_queue.Add(node_, true);
                            openset.Add(node_, g[node_] + (float)embedding.h(node_, goal_node));
                        }
                    }
                }
            }

        }

        if (parent.ContainsKey(goal_node))
        {
            current_node = goal_node;
            node_path.Add(current_node);
            path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));
            while (parent[current_node].Item1 != -1 || parent[current_node].Item2 != -1)
            {
                current_node = parent[current_node];
                node_path.Add(current_node);
                path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));

            }

            // start node - not to leave it out

            // node_path.Add(current_node); 
            // path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));

            path.Reverse();
            node_path.Reverse();
            var paths_finished = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
            return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_finished, true);
        }
        else
        {
            Debug.Log("Path not found, no adjacent nodes.");
            var paths_no_adj = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
            return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_no_adj, false);
        }

    }


    private static Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool> bfs(Vector3 start_pos, Vector3 goal_pos, GraphEmbedding embedding, TerrainInfo terrain_info)
    {
        Tuple<int, int> start_node = new Tuple<int, int>(embedding.get_i_index(start_pos[0], terrain_info),
                                                       embedding.get_j_index(start_pos[2], terrain_info)
                                                       );
        Tuple<int, int> goal_node = new Tuple<int, int>(embedding.get_i_index(goal_pos[0], terrain_info),
                                                      embedding.get_j_index(goal_pos[2], terrain_info)
                                                      );

        //List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> visited_nodes = new List<Tuple<int, int>>();
        List<Tuple<int, int>> to_visit = new List<Tuple<int, int>>();
        List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> node_path = new List<Tuple<int, int>>();
        Tuple<int, int> current_node;
        Dictionary<Tuple<int, int>, Tuple<int, int>> parent = new Dictionary<Tuple<int, int>, Tuple<int, int>>();

        //path.Add(start_pos);
        visited_nodes.Add(start_node);
        parent.Add(start_node, new Tuple<int, int>(-1, -1)); // root node of path

        foreach (Tuple<int, int> node in embedding.embedding[start_node])
        {
            to_visit.Add(node);
            parent.Add(node, start_node);
            visited_nodes.Add(node);
        }

        while (!(parent.ContainsKey(goal_node)) && to_visit.Count > 0)
        {
            current_node = to_visit[0];
            to_visit.RemoveAt(0);

            var adjacent_nodes = embedding.embedding[current_node];

            // remove visited
            adjacent_nodes.RemoveAll(l => visited_nodes.Contains(l));

            foreach (Tuple<int, int> node_ in adjacent_nodes)
            {
                to_visit.Add(node_);
                parent.Add(node_, current_node);
                visited_nodes.Add(node_);
            }

        }

        if (parent.ContainsKey(goal_node))
        {
            current_node = goal_node;
            node_path.Add(current_node);
            path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));
            while (parent[current_node].Item1 != -1 || parent[current_node].Item2 != -1)
            {
                current_node = parent[current_node];
                node_path.Add(current_node);
                path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));

            }

            // start node - not to leave it out

            // node_path.Add(current_node); 
            // path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));

            path.Reverse();
            node_path.Reverse();
            var paths_finished = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
            return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_finished, true);
        }
        else
        {
            Debug.Log("Path not found, no adjacent nodes.");
            var paths_no_adj = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
            return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_no_adj, false);
        }

    }


    private static Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool> greedy_path(Vector3 start_pos, Vector3 goal_pos, GraphEmbedding embedding, TerrainInfo terrain_info)
    {
        Tuple<int, int> start_node = new Tuple<int, int>(embedding.get_i_index(start_pos[0], terrain_info),
                                                       embedding.get_j_index(start_pos[2], terrain_info)
                                                       );
        Tuple<int, int> goal_node = new Tuple<int, int>(embedding.get_i_index(goal_pos[0], terrain_info),
                                                      embedding.get_j_index(goal_pos[2], terrain_info)
                                                      );

        List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> node_path = new List<Tuple<int, int>>();

        path.Add(start_pos);
        node_path.Add(start_node);

        List<Tuple<int, int>> visited_nodes = new List<Tuple<int, int>>();

        var current_node = start_node;
        var max_iter = 1000;
        var i = 0;
        var memory = max_iter;

        while ((current_node.Item1 != goal_node.Item1) || (current_node.Item2 != goal_node.Item2))
        {

            var adjacent_nodes = embedding.embedding[current_node];

            // remove visited
            // adjacent_nodes.RemoveAll(l => visited_nodes.Contains(l));

            if (adjacent_nodes.Count == 0)
            {
                Debug.Log("Path not found, no adjacent nodes.");
                // Vector3 _old_wp_ = start_pos;
                // foreach (var wp in path){
                //     Debug.DrawLine(_old_wp_, wp, Color.blue, 100f);
                //     _old_wp_ = wp;
                // } 
                var paths_no_adj = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
                return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_no_adj, false);
            }

            System.Random rng = new System.Random();
            var best_node = adjacent_nodes[(int)rng.Next(0, adjacent_nodes.Count)];
            foreach (var node in adjacent_nodes)
            {

                if (node == goal_node)
                {
                    path.Add(goal_pos);
                    node_path.Add(goal_node);
                    var paths_goal = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
                    return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_goal, true);
                }

                if (embedding.h(node, goal_node) < embedding.h(best_node, goal_node))
                {
                    best_node = node;
                }

            }

            // go to node which has lowest h, save it
            current_node = best_node;
            visited_nodes.Add(current_node);

            path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));

            node_path.Add(current_node);

            i++;
            if (i > max_iter)
            {
                Debug.Log("Path not found, maximal number of iterations reached.");
                // return new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
                Vector3 _old_wp = start_pos;
                foreach (var wp in path)
                {
                    Debug.DrawLine(_old_wp, wp, Color.green, 100f);
                    _old_wp = wp;
                }
                var paths_max_iter = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
                return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_max_iter, false);
            }
            // if (i > memory){
            //     // pop the first visited node
            //     visited_nodes.RemoveAt(0);
            // }
        }

        path.Add(goal_pos);
        node_path.Add(goal_node);

        var paths_finished = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
        return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_finished, true);


    }

}