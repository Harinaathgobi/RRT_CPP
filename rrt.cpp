#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <cmath>
#include <algorithm>
using namespace std;



struct Node 
{
    float x;
    float y; 
    vector<float> path_x;
    vector<float> path_y;
    int parent = -1;


    Node (float _x, float _y) : x(_x), y (_y) {} 

    Node () {}
};



class RRT
{
private:

	vector <tuple<float, float, float>>  obs_;  

   	float min_rand_= -2.0;
    float max_rand_= 15.0;

    float expand_dis_=3.0;
    float path_res_=0.25;
    int goal_sample_rate_=5;
    size_t max_iter_=500;

    Node start_node_;

    Node goal_node_;

public:

    Node generateRandomNode()
    {
  		random_device rd; 
  		mt19937 gen(rd());  
  		uniform_int_distribution<> goal_dis(0, 100);  
		uniform_real_distribution<> point_dis(min_rand_, max_rand_);

  		Node n;
		if (goal_dis(gen) > goal_sample_rate_) 
		{
		    n.x = point_dis(gen);
		    n.y = point_dis(gen);
		 } 
		 else 
		 {
		    n.x = goal_node_.x;
		    n.y = goal_node_.y;
		  }

		  return n;

	}

    size_t nearestNodeIndex(vector<Node>& nodes, Node& query)
    {
	  auto it =min_element(nodes.begin(), nodes.end(), [&](Node& n1, Node& n2) {
	        float dx1 = n1.x - query.x;
	        float dy1 = n1.y - query.y;

	        float dx2 = n2.x - query.x;
	        float dy2 = n2.y - query.y;
	        	  

	        return hypot(dx1, dy1) < hypot(dx2, dy2);
	      });

	  return distance(nodes.begin(), it);

	}


    Node steer(Node& from_node, Node& to_node, float extend_length)
    {
	  Node new_node(from_node.x, from_node.y);
	  new_node.path_x.push_back(new_node.x);
	  new_node.path_y.push_back(new_node.y);

	  auto [d, theta] = calculateDistanceAndAngle(new_node, to_node);

	  if (extend_length > d) extend_length = d;

	  int n_expand = floor(extend_length / path_res_);

	  for (int i = 0; i < n_expand; ++i) {
	    new_node.x += path_res_ * cos(theta);
	    new_node.y += path_res_ * sin(theta);
	    new_node.path_x.push_back(new_node.x);
	    new_node.path_y.push_back(new_node.y);
	  }

	  auto [final_d, final_theta] = calculateDistanceAndAngle(new_node, to_node);
	  if (final_d <= path_res_) {
	    new_node.path_x.push_back(to_node.x);
	    new_node.path_y.push_back(to_node.y);
	  }
	  return new_node;

	}


    bool noCollision(Node& n)
    {
	  for (auto [ox, oy, r] : obs_) 
	  {
	    for (size_t i = 0; i < n.path_x.size(); ++i) {
	      float dx = ox - n.path_x[i];
	      float dy = oy - n.path_y[i];
	      float d = hypot(dx, dy);

	      if (d <= r) return false;
	    }
	  }

	  return true;
	}


    pair<vector<float>, vector<float>> generateFinalCourse(vector<Node>& node_list) 
	{
	  vector<float> rx, ry;
	  size_t goal_id = node_list.size() - 1;
	  Node node = node_list[goal_id];
	  while (node.parent != -1) {
	    rx.push_back(node.x);
	    ry.push_back(node.y);
	    node = node_list[node.parent];
	  }

	  return {rx, ry};
	}

    pair<float, float> calculateDistanceAndAngle(Node& start, Node& goal)
    {
	  float dx = goal.x - start.x;
	  float dy = goal.y - start.y;
	  float d = hypot(dx, dy);
	  float theta = atan2(dy, dx);
	  return {d, theta};

	}

    float calculateDistanceToGoal(Node& n)
    {
	  float dx = n.x - goal_node_.x;
	  float dy = n.y - goal_node_.y;
	  return hypot(dx, dy);
	}


	pair<vector<float>, vector<float>>  planning(float sx, float sy, float gx, float gy, vector <tuple<float, float, float>> & obs)

	{

		obs_=obs;

		cout<< "Planning Started\n";

		start_node_ = Node(sx, sy);

  		goal_node_ = Node(gx, gy);

  		vector<Node> node_list;
  		node_list.emplace_back(sx, sy);




		for (int i = 0; i < max_iter_; i++) 
		{
		    Node random_node = generateRandomNode();

		    //cout<< "random node"<< random_node.x<<random_node.y<<endl;

		    size_t nearest_ind = nearestNodeIndex(node_list, random_node);
		    Node nearest_node = node_list[nearest_ind];

		    Node new_node = steer(nearest_node, random_node, expand_dis_);
		    new_node.parent = nearest_ind;

		 if (noCollision(new_node)) node_list.push_back(new_node);


    	if (calculateDistanceToGoal(new_node) <= expand_dis_) 
    	{
      		Node final_node = steer(new_node, goal_node_, expand_dis_);

      	if (noCollision(final_node)) 
     	 {
        	auto [rx, ry] = generateFinalCourse(node_list);

        	cout<< "Final course generated"<<endl;

        	return {rx, ry};
      	 }
    	
    	}
  	}

  	return {{}, {}};

}


};


int main() 
{	
	float sx,sy,gx,gy;
	float x,y,r;
	int obs_num;

	vector <tuple<float, float, float>> obs = {
        {5, 5, 1}

    };



    cout<<"Enter start points x and y: ";
	cin>> sx>>sy;
    cout<<"Enter goal points x and y: ";
	cin>> gx>>gy;

	cout<<"Enter the number of additional Obstacles";
	cin>> obs_num;

	for(int k=0; k<obs_num;k++)
	{
			cout<<"Additional circle obstacle "<<k+1<<"Enter x , y , r ";
			cin>> x >> y >> r;
		    obs.push_back(make_tuple(x, y, r));
	}


    RRT rrt;
    cout<<"RRT initialized\n";

    auto [rx, ry] = rrt.planning(sx, sy, gx, gy,obs);


    int n=rx.size();

    for(int j=n-1;j>=0;j--)
    {
		cout<< rx[j]<<"\t"<<ry[j]<<endl;

    }

    return 0;
}