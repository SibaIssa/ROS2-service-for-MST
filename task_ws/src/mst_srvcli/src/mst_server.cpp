#include "rclcpp/rclcpp.hpp" // ROS 2 C++ Client Library
#include "mst_msgs/msg/mst_input.hpp" // Package dependency
#include "mst_msgs/srv/mst_output.hpp"  

#include <iostream>
#include <memory>
#include<bits/stdc++.h>
#include <string>

using namespace std;

// Creating shortcut for an integer pair
typedef pair<int, int> iPair;

// Structure to represent a graph
struct Graph
{
	int V, E;
	vector< pair<int, iPair> > edges;

	// Constructor
	Graph(int V, int E)
	{
		this->V = V;
		this->E = E;
	}

	// Utility function to add an edge
	void addEdge(int u, int v, int w)
	{
		edges.push_back({w, {u, v}});
	}

	// Function to find MST using Kruskal's
	// MST algorithm
	string kruskalMST();
};

// To represent Disjoint Sets
struct DisjointSets
{
	int *parent, *rnk;
	int n;

	// Constructor.
	DisjointSets(int n)
	{
		// Allocate memory
		this->n = n;
		parent = new int[n+1];
		rnk = new int[n+1];

		// Initially, all vertices are in
		// different sets and have rank 0.
		for (int i = 0; i <= n; i++)
		{
			rnk[i] = 0;

			//every element is parent of itself
			parent[i] = i;
		}
	}

	// Find the parent of a node 'u'
	// Path Compression
	int find(int u)
	{
		/* Make the parent of the nodes in the path
		from u--> parent[u] point to parent[u] */
		if (u != parent[u])
			parent[u] = find(parent[u]);
		return parent[u];
	}

	// Union by rank
	void merge(int x, int y)
	{
		x = find(x), y = find(y);

		/* Make tree with smaller height
		a subtree of the other tree */
		if (rnk[x] > rnk[y])
			parent[y] = x;
		else // If rnk[x] <= rnk[y]
			parent[x] = y;

		if (rnk[x] == rnk[y])
			rnk[y]++;
	}
};


/* Functions returns weight of the MST*/
string Graph::kruskalMST()
{
        string out;
	int mst_wt = 0; // Initialize result

	// Sort edges in increasing order on basis of cost
	sort(edges.begin(), edges.end());

	// Create disjoint sets
	DisjointSets ds(V);

	// Iterate through all sorted edges
	vector< pair<int, iPair> >::iterator it;
	for (it=edges.begin(); it!=edges.end(); it++)
	{
		int u = it->second.first;
		int v = it->second.second;

		int set_u = ds.find(u);
		int set_v = ds.find(v);

		// Check if the selected edge is creating
		// a cycle or not (Cycle is created if u
		// and v belong to same set)
		if (set_u != set_v)
		{
			// Current edge will be in the MST
			// so print it
                        out = out + char(u+'A') + "-" + char(v+'A') + " "; 

			// Update MST weight
			mst_wt += it->first;

			// Merge two sets
			ds.merge(set_u, set_v);
		}
	}

	return out;
}

int countDistinct(string s) 
{ 

    unordered_map<char, int> m; 

    for (int i = 0; i < s.length(); i++) { 
        m[s[i]]++; 
    } 

    return m.size(); 
} 


void mst(const std::shared_ptr<mst_msgs::srv::MstOutput::Request> request,
          std::shared_ptr<mst_msgs::srv::MstOutput::Response>      response)
{

    string tree= request->input;

    //Calculating the vertices
    int V = countDistinct(tree)-2;

    //Calculating the Edges
    int E = 0 ;
    for(int i=0; i<=tree[i]; i++)
    {
        if(tree[i]=='-')
        {
            E++;
            
        }
    }


    Graph g(V, E);
  
    // making above shown graph
    int i=0;
    int k=0;
    while(i<E)
    {
        g.addEdge((int)(tree[k]-'A'), (int)(tree[k+2]-'A'), 1);
        k=k+4;
        i=i+1;
    }

    string mst_wt = g.kruskalMST();

  // gives the MST to the response
  response->output = mst_wt;
   
  // Notifies the console of its status using logs.
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                request->input);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " ");
}
 
int main(int argc, char **argv)
{
  // Initialize the ROS 2 C++ Client Library
  rclcpp::init(argc, argv);
 
  // Create a node named mst_server
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("mst_server");
 
  // Create a service named mst_service and advertise it over the network (i.e. &mst method)
  rclcpp::Service<mst_msgs::srv::MstOutput>::SharedPtr service =
    node->create_service<mst_msgs::srv::MstOutput>("mst_service", &mst);
 
  // Display a log message when the service is ready
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\nReady to Recieve the Request from the Client and Send back the Response to the Client...\n ");
 
  // Make the service available.
  rclcpp::spin(node);
   
  // Call shutdown procedure when we are done
  rclcpp::shutdown();
}
