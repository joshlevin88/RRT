//#include "stdafx.h"
//#include <iostream>
//#include <stdlib.h>
//#include <math.h>
#include <Engine.h>
#include <time.h>
#include <deque>
#include <vector>
#include <algorithm>

#pragma comment ( lib, "libmat.lib" )
#pragma comment ( lib, "libmx.lib"  )
#pragma comment ( lib, "libmex.lib" )
#pragma comment ( lib, "libeng.lib" )

#define PI 3.14159265358979323846f

// Tree
typedef struct node{
	float coord[3];
	float hdg;
	int tr_deg;
	int zr;
	int type;
	float time;
	float cost;
	struct node* next;
	struct node* child;
	struct node* parent;
} node;

// Displacement
typedef struct disp{
	float length;
	float cost;
} disp;

// List for nearest sorting
typedef struct list{
    node* node;
	float nearness;
	float smart_nearness;
} list;

// World
typedef struct world{
	float x_max, y_max, z_max;
	int n_obs, n_goals;
	float** obs;
	float** goals;
	node* start;
} world;

typedef float(*ptr_to_DCM)[3][3];

// Constants
const float V = 5.0f; // Flight velocity
const float bf = 2.0f; // Buffer around obstacles
const int max_tr_deg = 60; // Max turning rate
const int max_zr = 0; // Max climb rate
const float coll_dist = 0.2f; // Gap between collision checks
const float ATA_dist = 9.0f; // Distance required to do ATA
const float t_td = 0.23f; // Time-delay constant
const float t_int = 0.5f; // Planning horizon interval
const float ts = 0.5f; // Maximum time between nodes
const float ts_max = 1.0f; // Maximum time for a primitive
const int near_max = 5; // Maximum number of next near attempts
const float sd = 20.0f; // Algorithm slow-down factor

// Global variables
std::vector<node*> comm_vec; // Vector of committed nodes
bool path_found = false; // Whether a feasible path has been found yet
world w; // Environment information

// Functions
node* new_node(float, float, float, float, int, int, int, float, float, node*);
void rand_node_coord(node*, node*, int);
node* add_sibling(node*, node*);
node* add_child(node*, node*);
void trim_end_states(node*, node*, int, int, float);
node* extend_tree(node*, node*, node*, int);
node* steer_an(node*, node*);
node* steer_agile(node*, int);
disp disp_info(node*, node*);
bool collision(node*);
void free_tree(node**);
std::vector<list> near(node*, node*, int);
bool compare(const list &, const list &);
bool smart_compare(const list &, const list &);
int tree_size(node*);
void add_to_near_vec(node*, node*, std::vector<list>*);
void prune_tree(node**, node*);
bool update_tree(node**, node*);
std::deque<node*> root_to_end(node*, node*);
void add_to_commit(node*);
ptr_to_DCM create_DCM(float, float, float);
float norm(node*, node*);
void create_world(int);
bool goal_reached(node*, node*, int);
void plot_line(node*, node*);
void plot_point(node*);

int main()
{
	// Plot
	bool plotting = true;

	// Create world
	create_world(2);

	// Initialize tree with two nodes
	node* root = w.start;
	node* second = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 1.0f, 0.0f, NULL);
	trim_end_states(second, root, 0, 0, 0.5f);
	add_child(root, second);
	add_to_commit(root);

	// Remaining declarations and initializations
	node* prim_root;
	bool done = false;
	int iter = 0;
	int gn = 0;
	node* goal = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);
	goal->coord[0] = w.goals[gn][0];
	goal->coord[1] = w.goals[gn][1];
	goal->coord[2] = w.goals[gn][2];
	node* rand = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);

	// If plotting
	if (plotting) plot_line(second->parent, second);

	while (!done){

		// Start timer
		time_t start, end;
		float elapsed;
		bool done_iter = false;
		start = time(NULL);

		while (!done_iter){

			// Keep track of real-time computation interval
			end = time(NULL);
			elapsed = static_cast<float>(difftime(end, start));
			if (elapsed >= t_int*sd){
				done_iter = true;
				break;
			}

			// Generate a random node (change coordinates of random node)
			rand_node_coord(rand, goal, iter);

			// Extend tree
			prim_root = extend_tree(root, rand, goal, gn);

			// Check if current goal has been reached
			if (goal_reached(prim_root, goal, gn)){
				// If intermediate goal, update goal node
				if (gn < (w.n_goals - 1)){
					gn++;
					goal->coord[0] = w.goals[gn][0];
					goal->coord[1] = w.goals[gn][1];
					goal->coord[2] = w.goals[gn][2];
					printf("Intermediate goal reached! Next goal node: [%.1f, %.1f, %.1f]\n", goal->coord[0], goal->coord[1], goal->coord[2]);
				}
				// If final goal, feasible path has been found
				else if (!path_found){
					path_found = true;
					printf("Feasible path found!\n");
				}
			} 

			// Plot in MATLAB
			if (plotting){
				if (prim_root != NULL){
					plot_line(prim_root->parent, prim_root);
					while (prim_root->child){
						prim_root = prim_root->child;
						plot_line(prim_root->parent, prim_root);
					}
				}
			}

			iter++;
		}

		// Print number of iterations and size of tree
		printf("Number of iterations: %d,  Size of tree: %d\n", iter, tree_size(root));

		// Update tree in accordance with aircraft's real-time motion
		done = update_tree(&root, goal);

		if (plotting) plot_point(root);
	}

	// Add cruise-to-hover maneuver at end
	node* C2H_end = steer_agile(comm_vec.back(), 3);
	add_to_commit(C2H_end);
}

// Allocate dynamic memory and initialize new node
node* new_node(float coord_x, float coord_y, float coord_z, float hdg, int tr_deg, int zr, int type, float time, float cost, node* parent)
{
	node* new_node = (node*)malloc(sizeof(node));

	if (new_node) {
		new_node->coord[0] = coord_x;
		new_node->coord[1] = coord_y;
		new_node->coord[2] = coord_z;
		new_node->hdg = hdg;
		new_node->tr_deg = tr_deg;
		new_node->zr = zr;
		new_node->type = type;
		new_node->time = time;
		new_node->cost = cost;
		new_node->next = NULL;
		new_node->child = NULL;
		new_node->parent = parent;
	}

	return new_node;
}

// Generate random node (revisit because this will create garbage)
void rand_node_coord(node* rand_node, node* goal_node, int iter)
{
	int bias_freq = 30;

	// Every bias_freq iterations, return goal node
	if (iter % bias_freq == 0) {
		rand_node->coord[0] = goal_node->coord[0];
		rand_node->coord[1] = goal_node->coord[1];
		rand_node->coord[2] = goal_node->coord[2];
	}
	else{
		rand_node->coord[0] = ((float)rand() / (float)(RAND_MAX)) * w.x_max;
		rand_node->coord[1] = ((float)rand() / (float)(RAND_MAX)) * w.y_max;
		rand_node->coord[2] = ((float)rand() / (float)(RAND_MAX)) * w.z_max;
	}
}

// Add node to tree
node* add_child(node* existing_parent, node* new_child)
{
	if (existing_parent == NULL) return NULL;

	if (existing_parent->child) return add_sibling(existing_parent->child, new_child);

	else{
		new_child->parent = existing_parent;
		return existing_parent->child = new_child;
	}
}

// Add node as sibling if parent node already has a child
node* add_sibling(node* existing_child, node* new_sibling)
{
	if (existing_child == NULL) return NULL;

	while (existing_child->next) existing_child = existing_child->next;

	new_sibling->parent = existing_child->parent;
	return existing_child->next = new_sibling;
}

// Calculate trim states (coordinates and heading) at end of primitive
void trim_end_states(node* current, node* from, int tr_deg, int zr, float dt)
{
	float tr_rad = -tr_deg*PI/180.0f;

	// If straight 
	if (tr_deg == 0){
		current->coord[0] = from->coord[0] + V*dt*cos(from->hdg)*cos(asin(zr/V));
		current->coord[1] = from->coord[1] + V*dt*sin(from->hdg)*cos(asin(zr / V));
		current->coord[2] = from->coord[2] + zr*dt;
	}
	// If turning
	else{
		current->coord[0] = from->coord[0] + (-V / tr_rad*sin(from->hdg - tr_rad*dt) + V / tr_rad*sin(from->hdg))*cos(asin(zr / V));
		current->coord[1] = from->coord[1] + (V / tr_rad*cos(from->hdg - tr_rad*dt) - V / tr_rad*cos(from->hdg))*cos(asin(zr / V));
		current->coord[2] = from->coord[2] + zr*dt;
	}

	// Calculate heading and keep between -PI and PI
	current->hdg = from->hdg - tr_rad*dt;
	current->hdg = fmod(current->hdg, 2 * PI);
	if (current->hdg > PI) current->hdg = -2 * PI + current->hdg;
	else if (current->hdg < -PI) current->hdg = 2 * PI + current->hdg; 
}

// Extend tree
node* extend_tree(node* root, node* rand_node, node* goal, int gn)
{
	std::vector<list> near_vec;
	node* prim_root;

	// Nearness criteria depends on whether feasible path has been found
	int n;
	if (!path_found) n = 0;
	else n = 0;

	// Get vector of nodes orderered by nearness
	near_vec = near(root, rand_node, n);
	
	// Try steering from nodes in order
	for (int i = 0, n = static_cast<int>(near_vec.size()); i < std::min(5,n); i++){

		if (i == 0) prim_root = steer_an(near_vec[0].node, rand_node);
		else if (i == 1) prim_root = steer_agile(near_vec[0].node, 2); // Make second attempt ATA
		else prim_root = steer_an(near_vec[i - 1].node, rand_node); // Then keep trying trim primitives

		// Check for collision
		if (!collision(prim_root)){
			if (i == 0) add_child(near_vec[0].node, prim_root);
			else add_child(near_vec[i - 1].node, prim_root);
			break;
		}

		// Deallocate memory if primitive causes collision
		else {
		    free_tree(&prim_root);
			prim_root = NULL;
		}
	}
	
	return prim_root;
}

// Generate maneuver primitive steering towards node
node* steer_an(node* from, node* towards)
{
	int tr_deg, zr;
	float L;
	node* temp;

	// First node of primtive is time-delayed node
	node* td = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 1, from->time + t_td, 0.0f, from);
	trim_end_states(td, from, from->tr_deg, from->zr, t_td);
	disp d = disp_info(from, td);
	td->cost = from->cost + d.cost;

	// Find best trim primitive and time spent along it
	float r_G = norm(td, towards);
	float theta_G = atan2(towards->coord[1] - td->coord[1], towards->coord[0] - td->coord[0]) - td->hdg;

	if (theta_G > PI) theta_G = -2 * PI + theta_G;
	else if (theta_G < -PI) theta_G = 2 * PI + theta_G;

	float r_c = sqrtf(powf(towards->coord[0] - td->coord[0], 2) + powf(towards->coord[1] - td->coord[1], 2)) / (2 * sin(theta_G));

	if (theta_G != 0) L = r_G*theta_G / sin(theta_G); 
	else L = r_G;

	float dt_max = std::min((L/V), ts_max); // Max time on primitive must not be greater than const ts_max
	float dt = std::min(ts, dt_max); // Time step must not be greater than dt_max;

	tr_deg = int(V / r_c * 180.0f / PI);
	tr_deg = ((tr_deg + 10 / 2) / 10) * 10; // Round turning rate to nearest 10 deg
	zr = int((towards->coord[2] - td->coord[2]) / dt_max);

	// Keep turning and climb/descent rate within allowable bounds
	if (tr_deg > max_tr_deg) tr_deg = max_tr_deg;
	else if (tr_deg < -max_tr_deg) tr_deg = -max_tr_deg;
	if (zr > max_zr) zr = max_zr;
	else if (zr < -max_zr) zr = -max_zr;

	// Assign primitive's turning and climb/descent rate to time-delayed node
	td->tr_deg = tr_deg;
	td->zr = zr;

	// Add intermediate nodes along trim primitive to tree
	node* last = td;
	do{
		temp = new_node(0.0f, 0.0f, 0.0f, 0.0f, tr_deg, zr, 0, td->time + dt, 0.0f, last);
		trim_end_states(temp, td, tr_deg, zr, dt);
		temp->cost = last->cost + disp_info(last, temp).cost;
		add_child(last, temp);

		last = temp;
		dt += ts;
	} while (dt <= dt_max);

	return td; // Time-delayed node is root of primitive
}

// Generate agile maneuver primitive 
node* steer_agile(node* from, int m_type)
{
	float dx_man[14], dy_man[14], dz_man[14], t_man[14];

	float dx_ATA[14] = { 1.3831f, 2.4070f, 3.2652f, 4.1171f, 4.7425f, 4.9805f, 5.0326f, 4.8323f, 4.2057f, 3.4872f, 2.6745f, 1.5137f, 0.4182f, 0.0f };
	float dy_ATA[14] = { 0.0041f, 0.0370f, 0.1092f, 0.2218f, 0.3066f, 0.3109f, 0.2524f, 0.1077f, -0.0235f, -0.0477f, -0.0339f, -0.0077f, -0.0002f, 0.0f };
	float dz_ATA[14] = { 0.0012f, 0.0373f, 0.1580f, 0.4257f, 0.7802f, 1.0111f, 1.1569f, 1.1725f, 0.8915f, 0.5674f, 0.2876f, 0.0526f, -0.0014f, 0.0f };
	float t_ATA[14] = { 0.1971f, 0.3439f, 0.4784f, 0.6487f, 0.8458f, 0.9926f, 1.1271f, 1.2974f, 1.4945f, 1.6413f, 1.7758f, 1.9461f, 2.1024f, 2.1623f };

	float dx_C2H[14] = { 0.2538f, 1.1928f, 2.2255f, 2.8981f, 3.4204f, 3.9541f, 4.4108f, 4.6521f, 4.8088f, 4.9322f, 4.9922f, 5.0000f, 4.9961f, 4.9930f };
	float dy_C2H[14] = { 0.0f };
	float dz_C2H[14] = { -0.0002f, 0.0007f, 0.0677f, 0.1874f, 0.3305f, 0.5262f, 0.7360f, 0.8648f, 0.9564f, 1.0360f, 1.0851f, 1.1011f, 1.1070f, 1.1084f };
	float t_C2H[14] = { 0.0362f, 0.1707f, 0.3263f, 0.4422f, 0.5483f, 0.6828f, 0.8384f, 0.9543f, 1.0604f, 1.1949f, 1.3505f, 1.4664f, 1.5725f, 1.7070f };

	// If ATA
	if (m_type == 2){
		for (int i = 0; i < 14; i++){
			dx_man[i] = dx_ATA[i];
			dy_man[i] = dy_ATA[i];
			dz_man[i] = dz_ATA[i];
			t_man[i] = t_ATA[i];
		}
	}

	// If C2H
	else if (m_type == 3){
		for (int i = 0; i < 14; i++){
			dx_man[i] = dx_C2H[i];
			dy_man[i] = dy_C2H[i];
			dz_man[i] = dz_C2H[i];
			t_man[i] = t_C2H[i];
		}
	}

	// Rotate to align with heading at from node
	ptr_to_DCM DCM = create_DCM(0.0f, 0.0f, from->hdg);
	for (int i = 0; i < 14; i++){
		dx_man[i] = (*DCM)[0][0] * dx_man[i] + (*DCM)[0][1] * dy_man[i] + (*DCM)[0][2] * dz_man[i];
		dy_man[i] = (*DCM)[1][0] * dx_man[i] + (*DCM)[1][1] * dy_man[i] + (*DCM)[1][2] * dz_man[i];
		dz_man[i] = (*DCM)[2][0] * dx_man[i] + (*DCM)[2][1] * dy_man[i] + (*DCM)[2][2] * dz_man[i];
	}
	free(DCM);

	// Root of primtive is time-delayed node
	node* td = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 1, from->time + t_td, 0.0f, from);
	trim_end_states(td, from, from->tr_deg, from->zr, t_td);
	td->cost = from->cost + disp_info(from, td).cost;

	// End node
	node* end = new_node(td->coord[0] + dx_man[14], td->coord[1] + dy_man[14], td->coord[2] + dz_man[14], 0.0f, 0, 0, m_type, td->time + t_man[14], 0.0f, td);
	end->cost = td->cost + 50.0f;
	end->hdg = td->hdg + PI;

	// Make sure heading lies between -PI and PI
	end->hdg = fmod(end->hdg, 2 * PI); 
	if (end->hdg > PI) end->hdg = -2 * PI + end->hdg;
	else if (end->hdg < -PI) end->hdg = 2 * PI + end->hdg;

	add_child(td, end); // Append end node to time-delayed node

	return td; // Time-delayed node is root of primitive
}

// Update tree in accordance with aircraft's real-time motion
bool update_tree(node** root, node* goal)
{ 
	bool done = false; // Has the aircraft reached the end of the trajectory

	// Find node nearest goal via smart nearness criteria
	std::vector<list> nearest_vec = near(*root, goal, 0);
	node* end = nearest_vec[0].node;

	// Get deque of nodes after current root to that nearest node
	std::deque<node*> r2e_list = root_to_end(*root, end);

	// Get node at or just past time interval on way to nearest node
	node* comm_end = *root;
	while ((comm_end->time - (*root)->time) < t_int){

		// If no more nodes to move towards, algorithm is done
		if (r2e_list.empty()){
			if (!path_found) printf("Caught up to trajectory end before reaching goal :(\n");
			else printf("Goal node reached :)\n");
			return done = true;
		}

		comm_end = r2e_list.front();	
		add_to_commit(comm_end);

		r2e_list.pop_front();
	}

	// Prune tree
	prune_tree(root, comm_end);
	*root = comm_end;
	(*root)->parent = NULL;

	return done;
}

// Displacement information for trajectory (length and cost)
disp disp_info(node* from, node* to)
{
	float L;

	float r = norm(from, to);
	float theta = atan2f(to->coord[1] - from->coord[1], to->coord[0] - from->coord[0]) - from->hdg;

	if (theta > PI) theta = -2 * PI + theta;
	else if (theta < -PI) theta = 2 * PI + theta;

	if (theta != 0) L = r*theta / sin(theta);
	else L = r;

	disp d = {L, L};

	return d;
}

// Check for collision
bool collision(node* root)
{
	bool collision = false;
	node* from = root;
	node* to;
	node* temp = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 1.0f, 0.0f, NULL);
	float dt, dt_end;

	while (from->child){

		to = from->child;

		// Check if node is in world
		if (to->coord[0] < 0.0f || to->coord[0] > w.x_max || to->coord[1] < 0.0f || to->coord[1] > w.y_max || to->coord[2] < 0.0f || to->coord[2] > w.z_max){
			free(temp);
			return collision = true;
		}

		dt = coll_dist / V; // Check for collision every coll_dist

		if (to->type < 2) dt_end = (to->time - from->time); 
		else dt_end = 5.0f / V; // Agile maneuvers take up 5 metres ahead of previous trajectory

		// Check if path to node collides with obstacle
		while (dt <= dt_end){
			trim_end_states(temp, from, to->tr_deg, to->zr, dt);
			for (int i = 0; i < w.n_obs; i++){
				if (temp->coord[0] >= (w.obs[i][0] - bf) &&
					temp->coord[0] <= (w.obs[i][0] + w.obs[i][3] + bf) &&
					temp->coord[1] >= (w.obs[i][1] - bf) &&
					temp->coord[1] <= (w.obs[i][1] + w.obs[i][4] + bf) &&
					temp->coord[2] >= (w.obs[i][2] - bf) &&
					temp->coord[2] <= (w.obs[i][2] + w.obs[i][5] + bf)){
					free(temp);
					return collision = true;
				}
			}
			dt += coll_dist / V;
		}
		from = to;
	}

	free(temp);
	return collision;
}

// Deallocate memory of tree
void free_tree(node** n)
{
	if (*n == NULL)
		return;

	free_tree(&((*n)->child));
	free_tree(&((*n)->next));

	free(*n);
	*n = NULL;
}

// Prune tree (free memory of nodes that aren't in new_root's tree)
void prune_tree(node** root, node* new_root)
{
	bool tbd = true;

	if (*root == NULL)
		return;

	if (*root == new_root) {
		tbd = false;
	}

	if (tbd){
		prune_tree(&(*root)->child, new_root);
		prune_tree(&(*root)->next, new_root);
		free(*root);
		*root = NULL;
	}
	else{
		prune_tree(&(*root)->next, new_root);
	}
}

// Create list of nodes in order of some nearness criteria
std::vector<list> near(node* root, node* to, int near_type)
{
	std::vector<list> near_vec; // Vector of nodes with nearness information
	add_to_near_vec(root, to, &near_vec); // Add all nodes in tree, with nearness information, to vector
	if (near_type == 0) std::sort(near_vec.begin(), near_vec.end(), compare); // Sort via Euclidian distance to to node
	else std::sort(near_vec.begin(), near_vec.end(), smart_compare); // Sort via smarter nearness criteria

	return near_vec;
}

// Euclidean distance comparison
bool compare(const list &a, const list &b)
{
	return a.nearness < b.nearness;
}

// Smarter nearness comparison
bool smart_compare(const list &a, const list &b)
{
	return a.smart_nearness < b.smart_nearness;
}

// Get size of tree
int tree_size(node* root)
{
	if (root == NULL) 
		return 0;
	else{
		return (tree_size(root->child) + 1 + tree_size(root->next));
	}
}

// Add nodes in tree, with nearness information, to vector
void add_to_near_vec(node* n, node* to, std::vector<list>* near_vec)
{
	if (n == NULL) return;

	// Nearness info is based on distance from time-delayed node to to node
	node* td = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 1, n->time + t_td, 0.0f, n);
	trim_end_states(td, n, n->tr_deg, n->zr, t_td);
	td->cost = n->cost + disp_info(n, td).cost;

	float D = sqrtf(powf(td->coord[0] - w.start->coord[0], 2) + powf(td->coord[1] - w.start->coord[1], 2) + powf(td->coord[2] - w.start->coord[2], 2));
	float L = td->cost;
	float D2G = disp_info(td, to).length;

	// Node address and nearness info
	list l;
	l.node = n;
	l.nearness = disp_info(td, to).length;
	l.smart_nearness = (L + 2*D2G) / D;
	
	(*near_vec).push_back(l); // Add to vector

	free(td);

	// Recursively iterate through function for each node
	if (n->child != NULL) add_to_near_vec(n->child, to, near_vec);
	if (n->next != NULL) add_to_near_vec(n->next, to, near_vec);

	return;
}

// Create deque of nodes starting right after root and going to end
std::deque<node*> root_to_end(node* root, node* end)
{
	node* temp = end;
	std::deque<node*> r2e_dq(1, temp); // Initialize deque with end node
	while (temp->parent != root && temp->parent != NULL){
		temp = temp->parent;
		r2e_dq.push_front(temp); // Add to front of deque
	}

	return r2e_dq;
}

// Add node to vector of committed nodes
void add_to_commit(node* n)
{
	node* new_n = (node*)malloc(sizeof(node)); // Must make new copies of nodes because most will get pruned (i.e. deallocated)
	memcpy(new_n, n, sizeof(node)); 
	comm_vec.push_back(new_n); // Add to end of vector
}

// Create Direction Cosine Matrix
ptr_to_DCM create_DCM(float phi, float the, float psi)
{
	ptr_to_DCM DCM = (ptr_to_DCM)malloc(sizeof(*DCM));

	(*DCM)[0][0] = cos(the)*cos(psi);
	(*DCM)[0][1] = cos(the)*sin(psi);
	(*DCM)[0][2] = -sin(the);

	(*DCM)[1][0] = -cos(phi)*sin(psi) + sin(phi)*sin(the)*cos(psi);
	(*DCM)[1][1] = cos(phi)*cos(psi) + sin(phi)*sin(the)*sin(psi);
	(*DCM)[1][2] = sin(phi)*cos(the);

	(*DCM)[2][0] = sin(phi)*sin(psi) + cos(phi)*sin(the)*cos(psi);
	(*DCM)[2][1] = -sin(phi)*cos(psi) + cos(phi)*sin(the)*sin(psi);
	(*DCM)[2][2] = cos(phi)*cos(the);

	return DCM;
}

// Calculate L2-norm between two nodes
float norm(node* n1, node* n2)
{
	return sqrtf(powf(n2->coord[0] - n1->coord[0], 2) + powf(n2->coord[1] - n1->coord[1], 2) + powf(n2->coord[2] - n1->coord[2], 2));
}

// Create world
void create_world(int n)
{
	// Two columns
	if (n == 0){

		// Starting node
		w.start = new_node(5.0f, 4.0f, 5.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);

		// Environment boundaries
		w.x_max = 50.0f;
		w.y_max = 25.0f;
		w.z_max = 10.0f;

		// Obstacles
		w.n_obs = 2;
		float obs_arr[2][6] = { { 0.0f, 8.0f, 0.0f, 40.0f, 2.0f, 10.0f }, 
								{ 10.0f, 16.0f, 0.0f, 40.0f, 2.0f, 10.0f } };

		w.obs = (float**)malloc(w.n_obs * sizeof(float**));
		for (int i = 0; i < w.n_obs; i++){
			w.obs[i] = (float*)malloc(6 * sizeof(float*));
		}

		for (int i = 0; i < w.n_obs; i++){
			for (int j = 0; j < 6; j++){
				w.obs[i][j] = obs_arr[i][j];
			}
		}

		// Goals (intermediate and final)
		w.n_goals = 3;
		float goal_arr[3][5] = { {45.0f, 8.5f, 5.0f, PI/2.0f, 1.5f},
								 {5.0f, 16.5f, 5.0f, PI/2.0f, 1.5f},
								 {50.0f, 21.0f, 5.0f, 0.0f, 1.5f} };

		w.goals = (float**)malloc(w.n_goals * sizeof(float**));
		for (int i = 0; i < w.n_goals; i++){
			w.goals[i] = (float*)malloc(5 * sizeof(float*));
		}

		for (int i = 0; i < w.n_goals; i++){
			for (int j = 0; j < 5; j++){
				w.goals[i][j] = goal_arr[i][j];
			}
		}
	}
	
	// Two corridors
	else if (n == 1){

		w.start = new_node(1.0f, 1.0f, 5.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);

		w.x_max = 50.0f;
		w.y_max = 25.0f;
		w.z_max = 10.0f;

		w.n_obs = 2;
		float obs_arr[2][6] = { { 10.0f, 10.0f, 0.0f, 10.0f, 10.0f, 10.0f },
								{ 30.0f, 13.0f, 0.0f, 10.0f, 10.0f, 10.0f } };

		w.obs = (float**)malloc(w.n_obs * sizeof(float**));
		for (int i = 0; i < w.n_obs; i++){
			w.obs[i] = (float*)malloc(6 * sizeof(float*));
		}

		for (int i = 0; i < w.n_obs; i++){
			for (int j = 0; j < 6; j++){
				w.obs[i][j] = obs_arr[i][j];
			}
		}

		w.n_goals = 1;
		float goal_arr[1][5] = { 49.0f, 24.0f, 5.0f, PI / 2.0f, 2.0f };

		w.goals = (float**)malloc(w.n_goals * sizeof(float**));
		for (int i = 0; i < w.n_goals; i++){
			w.goals[i] = (float*)malloc(5 * sizeof(float*));
		}

		for (int i = 0; i < w.n_goals; i++){
			for (int j = 0; j < 5; j++){
				w.goals[i][j] = goal_arr[i][j];
			}
		}
	}

	// Wall with passage
	else {

		w.start = new_node(1.0f, 1.0f, 5.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);

		w.x_max = 50.0f;
		w.y_max = 25.0f;
		w.z_max = 10.0f;

		w.n_obs = 2;
		float obs_arr[2][6] = { { 24.0f, 0.0f, 0.0f, 2.0f, 10.0f, 10.0f },
								{ 24.0f, 15.0f, 0.0f, 2.0f, 10.0f, 10.0f } };

		w.obs = (float**)malloc(w.n_obs * sizeof(float**));
		for (int i = 0; i < w.n_obs; i++){
			w.obs[i] = (float*)malloc(6 * sizeof(float*));
		}

		for (int i = 0; i < w.n_obs; i++){
			for (int j = 0; j < 6; j++){
				w.obs[i][j] = obs_arr[i][j];
			}
		}

		w.n_goals = 2;
		float goal_arr[2][5] = { { 25.0f, 12.5f, 5.0f, 0.0f, 1.0f },
							     { 48.0f, 23.0f, 5.0f, 0.0f, 1.0f } };

		w.goals = (float**)malloc(w.n_goals * sizeof(float**));
		for (int i = 0; i < w.n_goals; i++){
			w.goals[i] = (float*)malloc(5 * sizeof(float*));
		}

		for (int i = 0; i < w.n_goals; i++){
			for (int j = 0; j < 5; j++){
				w.goals[i][j] = goal_arr[i][j];
			}
		}
	}
}

// Check if goal region has been reached
bool goal_reached(node* n, node* goal, int gn)
{
	if (n == NULL) return false;
	if (norm(n, goal) <= w.goals[gn][4]) return true; // If in region
	if (goal_reached(n->next, goal, gn)) return true; // Recursively iterate
	return goal_reached(n->child, goal, gn);
}

// Plot line in MATLAB
void plot_line(node* from, node* to)
{
	Engine *m_eng = engOpen("null");

	mxArray* from_x = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* from_y = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* to_x = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* to_y = mxCreateDoubleMatrix(1, 1, mxREAL);

	double* p_from_x = mxGetPr(from_x);
	double* p_from_y = mxGetPr(from_y);
	double* p_to_x = mxGetPr(to_x);
	double* p_to_y = mxGetPr(to_y);

	*p_from_x = (double)from->coord[0];
	*p_from_y = (double)from->coord[1];
	*p_to_x = (double)to->coord[0];
	*p_to_y = (double)to->coord[1];

	engPutVariable(m_eng, "from_x", from_x);
	engPutVariable(m_eng, "from_y", from_y);
	engPutVariable(m_eng, "to_x", to_x);
	engPutVariable(m_eng, "to_y", to_y);

	engEvalString(m_eng, "line([from_x to_x],[from_y to_y]), set(gca, 'XLim', [0 50], 'YLim', [0 25]); hold on;");
}

// Plot point in MATLAB
void plot_point(node* current)
{
	Engine *m_eng = engOpen("null");

	mxArray* curr_x = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* curr_y = mxCreateDoubleMatrix(1, 1, mxREAL);

	double* p_curr_x = mxGetPr(curr_x);
	double* p_curr_y = mxGetPr(curr_y);

	*p_curr_x = (double)current->coord[0];
	*p_curr_y = (double)current->coord[1];

	engPutVariable(m_eng, "curr_x", curr_x);
	engPutVariable(m_eng, "curr_y", curr_y);

	engEvalString(m_eng, "plot(curr_x, curr_y, 'r.', 'MarkerSize', 12); hold on;");
}

/*
node* steer(node* from, node* towards)
{
	// Root of primtive is time-delayed node
	node* td = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 1, from->time + t_td, 0.0f, from);
	trim_end_states(td, from, from->tr_deg, from->zr, t_td);
	disp d = disp_info(from, td);
	td->cost = d.cost;

	node* temp = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);
	disp d2 = disp_info(td, towards);
	float L = d.length + d2.length;
	float dt;
	int tr_deg;
	int zr;
	float min_norm = 1000;
	float norm_temp;
	int i = 0;
	int j, k;
	float dt_i = ts;

	// Find best trim primitive and time spent along it
	do{
		i++;
		j = 0;
		for (int tr_deg_i = -max_tr_deg; tr_deg_i <= max_tr_deg; tr_deg_i += 10){
			j++;
			k = 0;
			for (int zr_i = -max_zr; zr_i <= max_zr; zr_i += 1){
				k++;
				trim_end_states(temp, from, tr_deg_i, zr_i, dt_i);
				norm_temp = norm(temp, towards);
				if (norm_temp < min_norm){
					min_norm = norm_temp;
					tr_deg = tr_deg_i;
					zr = zr_i;
					dt = dt_i;
				}
			}
		}
		dt_i += ts;
	} while (dt_i <= L / V);

	free(temp);
	if (dt > ts_max) dt = ts_max;

	td->tr_deg = tr_deg;
	td->zr = zr;

	node* last = td;
	dt_i = ts;
	if (ts > dt) dt_i = dt;

	// Add intermediate nodes along trim primitive to tree
	do{
		temp = new_node(0.0f, 0.0f, 0.0f, 0.0f, tr_deg, zr, 0, td->time + dt_i, 0.0f, last);
		trim_end_states(temp, td, tr_deg, zr, dt_i);
		temp->cost = disp_info(last, temp).cost;
		add_child(last, temp);

		last = temp;
		dt_i += ts;
	} while (dt_i <= dt);

	return td;
}
*/


