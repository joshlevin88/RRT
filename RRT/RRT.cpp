#include "stdafx.h"
#include <Engine.h>
#include<iostream>
#include<math.h>
#include<time.h>
#include<deque>
#include<vector>

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
	struct node* node;
	float nearness;
	float smart_nearness;
} list;

typedef float(*ptr_to_DCM)[3][3];

// Constants
const float V = 5.0f;
const float ts = 0.25f;
const float bf = 1.5f;
const int max_tr_deg = 60;
const int max_zr = 1;
const float coll_dist = 0.2f;
const float ATA_dist = 9.0f;
const float t_td = 0.23f;
const float t_int = 0.5f;
const float dt_max = 1.0f;
const int near_max = 5;
const float x_max = 50.0f;
const float y_max = 30.0f;
const float z_max = 10.0f;
const float sd = 20.0f;
const float start_coord[3] = { 1.0f, 1.0f, 1.0f };
std::vector<node*> comm_vec;

// Functions
node* new_node(float, float, float, float, int, int, int, float, float, node*);
void rand_node_coord(node*, node*, int);
node* add_sibling(node*, node*);
node* add_child(node*, node*);
void trim_end_states(node*, node*, int, int, float);
node* extend_tree(node*, node*);
node* steer_an(node*, node*);
node* steer_agile(node*, int);
disp disp_info(node*, node*);
bool collision(node*);
void free_tree(node**);
list* near(node*, node*, int);
int compare(const void*, const void*);
int smart_compare(const void*, const void*);
int tree_size(node*);
int add_to_array(node*, node*, list*, int);
void prune_tree(node**, node*);
bool update_tree(node**, node*);
std::deque<node*> root_to_end(node*);
void add_to_commit(node*);
ptr_to_DCM create_DCM(float, float, float);

/*
node* steer(node*, node*);
vector<float> generateRange(float, float, float);
vector<float> linspace(float, float, int);
node* search_tree(node*, int);
node* compare_tree(node*);
void backwards(node*);
*/


int main()
{
	// For plotting in MATLAB
	
	Engine *m_eng;
	m_eng = engOpen("null");
	mxArray* from_x = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* from_y = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* to_x = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* to_y = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* curr_x = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* curr_y = mxCreateDoubleMatrix(1, 1, mxREAL);
	double* p_from_x = mxGetPr(from_x);
	double* p_from_y = mxGetPr(from_y);
	double* p_to_x = mxGetPr(to_x);
	double* p_to_y = mxGetPr(to_y);
	double* p_curr_x = mxGetPr(curr_x);
	double* p_curr_y = mxGetPr(curr_y);
	
	node* prim_root;
	

	// Initialize tree with two nodes
	node* root = new_node(start_coord[0], start_coord[1], start_coord[2], PI/4, 0, 0, 0, 0.0f, 0.0f, NULL);
	node* second = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 1.0f, 0.0f, NULL);
	trim_end_states(second, root, 0, 0, 1.0f);
	add_child(root, second);
	add_to_commit(root);

	*p_from_x = (double)second->parent->coord[0];
	*p_from_y = (double)second->parent->coord[1];
	*p_to_x = (double)second->coord[0];
	*p_to_y = (double)second->coord[1];
	engPutVariable(m_eng, "from_x", from_x);
	engPutVariable(m_eng, "from_y", from_y);
	engPutVariable(m_eng, "to_x", to_x);
	engPutVariable(m_eng, "to_y", to_y);
	engEvalString(m_eng, "line([from_x to_x],[from_y to_y]), set(gca, 'XLim', [0 50], 'YLim', [0 30]); hold on;");

	//bool path_found = false;
	bool done = false;
	int iter = 0;
	//node* commited_root = (node*)malloc(sizeof(node));
	//commited_root = root;
	//int gn = 1;
	node* goal = new_node(45.0f, 25.0f, 5.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);
	node* rand = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);

	while (!done){

		time_t start, end;
		double elapsed;
		bool done_iter = false;

		start = time(NULL);
		while (!done_iter){

			// Keep track of real-time computation interval
			end = time(NULL);
			elapsed = difftime(end, start);
			if (elapsed >= t_int*sd){
				done_iter = true;
				break;
			}

			// Generate a random node
			rand_node_coord(rand, goal, iter);

			// Extend tree
			prim_root = extend_tree(root, rand);

			// Plot in MATLAB
			
			if (prim_root != NULL){

				*p_from_x = (double)prim_root->parent->coord[0];
				*p_from_y = (double)prim_root->parent->coord[1];
				*p_to_x = (double)prim_root->coord[0];
				*p_to_y = (double)prim_root->coord[1];
				engPutVariable(m_eng, "from_x", from_x);
				engPutVariable(m_eng, "from_y", from_y);
				engPutVariable(m_eng, "to_x", to_x);
				engPutVariable(m_eng, "to_y", to_y);
				engEvalString(m_eng, "line([from_x to_x],[from_y to_y]), set(gca, 'XLim', [0 50], 'YLim', [0 30]); hold on;");

				while (prim_root->child){

					prim_root = prim_root->child;

					*p_from_x = (double)prim_root->parent->coord[0];
					*p_from_y = (double)prim_root->parent->coord[1];
					*p_to_x = (double)prim_root->coord[0];
					*p_to_y = (double)prim_root->coord[1];
					*p_curr_x = (double)root->coord[0];
					*p_curr_y = (double)root->coord[1];
					engPutVariable(m_eng, "from_x", from_x);
					engPutVariable(m_eng, "from_y", from_y);
					engPutVariable(m_eng, "to_x", to_x);
					engPutVariable(m_eng, "to_y", to_y);
					engPutVariable(m_eng, "curr_x", curr_x);
					engPutVariable(m_eng, "curr_y", curr_y);
					engEvalString(m_eng, "line([from_x to_x],[from_y to_y]), set(gca, 'XLim', [0 50], 'YLim', [0 30]); hold on;");
					engEvalString(m_eng, "plot(curr_x, curr_y, 'r.', 'MarkerSize', 12); hold on;");
				}
			}
			
			iter++;
		}
		done = update_tree(&root, goal);
		//printf("Root time: %.2f\n", root->time);
		printf("Iterations: %d\n", iter);
		printf("Size of tree: %d\n", tree_size(root));
	}
	for (int i = 0; i < comm_vec.size(); i++)
		printf("comm_vec[%d]->time: %.2f\n", i, comm_vec[i]->time);
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
	int bias_freq = 100;

	// Every bias_freq iterations, return goal node
	if (iter % bias_freq == 0) {
		rand_node->coord[0] = goal_node->coord[0];
		rand_node->coord[1] = goal_node->coord[1];
		rand_node->coord[2] = goal_node->coord[2];
	}
	else{
		rand_node->coord[0] = ((float)rand() / (float)(RAND_MAX)) * x_max;
		rand_node->coord[1] = ((float)rand() / (float)(RAND_MAX)) * y_max;
		rand_node->coord[2] = ((float)rand() / (float)(RAND_MAX)) * z_max;
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
node* add_sibling(node* existing_child, node* new_next)
{
	if (existing_child == NULL) return NULL;

	while (existing_child->next) existing_child = existing_child->next;

	new_next->parent = existing_child->parent;
	return existing_child->next = new_next;
}

// Calculate trim states (coordinates and heading) at end of trim primitive
void trim_end_states(node* current, node* from, int tr_deg, int zr, float dt)
{
	float tr_rad = tr_deg*PI/180.0f;

	if (tr_deg == 0){
		current->coord[0] = from->coord[0] + V*dt*cos(from->hdg)*cos(asin(zr/V));
		current->coord[1] = from->coord[1] + V*dt*sin(from->hdg)*cos(asin(zr / V));
		current->coord[2] = from->coord[2] + zr*dt;
	}
	else{
		current->coord[0] = from->coord[0] + (-V / tr_rad*sin(from->hdg - tr_rad*dt) + V / tr_rad*sin(from->hdg))*cos(asin(zr / V));
		current->coord[1] = from->coord[1] + (V / tr_rad*cos(from->hdg - tr_rad*dt) - V / tr_rad*cos(from->hdg))*cos(asin(zr / V));
		current->coord[2] = from->coord[2] + zr*dt;
	}

	current->hdg = from->hdg - tr_rad*dt;
	current->hdg = fmod(current->hdg, 2 * PI);
	if (current->hdg > PI) current->hdg = -2 * PI + current->hdg;
	else if (current->hdg < -PI) current->hdg = 2 * PI + current->hdg; 
}

// Extend tree
node* extend_tree(node* root, node* rand_node)
{
	list* nearest_list = near(root, rand_node, 0);
	node* prim_root;
	for (int i = 0; i <= 4; i++){

		if (i == 0) prim_root = steer_an(nearest_list[i].node, rand_node);
		else if (i == 1) prim_root = steer_agile(nearest_list[0].node, 2);
		else if (i > 1) prim_root = steer_an(nearest_list[i-1].node, rand_node);

		if (collision(prim_root) == false){
			add_child(nearest_list[i].node, prim_root);
			break;
		}
		else {
		    free_tree(&prim_root);
			prim_root = NULL;
		}
	}
	
	free(nearest_list);
	return prim_root;
}

node* steer_an(node* from, node* towards)
{
	int tr_deg;
	int zr;
	node* temp;

	// Root of primtive is time-delayed node
	node* td = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 1, from->time + t_td, 0.0f, from);
	trim_end_states(td, from, from->tr_deg, from->zr, t_td);
	disp d = disp_info(from, td);
	td->cost = d.cost;

	// Find best trim primitive and time spent along it
	float r_G = sqrtf(powf(towards->coord[0] - td->coord[0], 2) + powf(towards->coord[1] - td->coord[1], 2) + powf(towards->coord[2] - td->coord[2], 2));
	float theta_G = atan2(towards->coord[1] - td->coord[1], towards->coord[0] - td->coord[0]) - td->hdg;

	if (theta_G > PI) theta_G = -2 * PI + theta_G;
	else if (theta_G < -PI) theta_G = 2 * PI + theta_G;

	float r_c = sqrtf(powf(towards->coord[0] - td->coord[0], 2) + powf(towards->coord[1] - td->coord[1], 2)) / (2 * sin(theta_G));
	float L;

	if (theta_G != 0) L = r_G*theta_G / sin(theta_G); 
	else L = r_G;

	float dt = L / V;
	if (dt > dt_max) dt = dt_max;

	tr_deg = int(V / r_c * 180.0f / PI);
	tr_deg = ((tr_deg + 10 / 2) / 10) * 10;
	zr = int((towards->coord[2] - td->coord[2]) / dt);

	if (tr_deg > max_tr_deg) tr_deg = max_tr_deg;
	else if (tr_deg < -max_tr_deg) tr_deg = -max_tr_deg;
	if (zr > max_zr) zr = max_zr;
	else if (zr < -max_zr) zr = -max_zr;

	td->tr_deg = tr_deg;
	td->zr = zr;

	node* last = td;
	float dt_i = ts;
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

	if (m_type == 2){
		for (int i = 0; i < 14; i++){
			dx_man[i] = dx_ATA[i];
			dy_man[i] = dy_ATA[i];
			dz_man[i] = dz_ATA[i];
			t_man[i] = t_ATA[i];
		}
	}
	else if (m_type == 3){
		for (int i = 0; i < 14; i++){
			dx_man[i] = dx_C2H[i];
			dy_man[i] = dy_C2H[i];
			dz_man[i] = dz_C2H[i];
			t_man[i] = t_C2H[i];
		}
	}

	// Rotate by heading at from node
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
	disp d = disp_info(from, td);
	td->cost = d.cost;

	// End node
	node* end = new_node(td->coord[0] + dx_man[14], td->coord[1] + dy_man[14], td->coord[2] + dz_man[14], 0.0f, 0, 0, m_type, td->time + t_man[14], 0.0f, td);
	end->cost = disp_info(td, end).cost;
	end->hdg = td->hdg + PI;
	end->hdg = fmod(end->hdg, 2 * PI);
	if (end->hdg > PI) end->hdg = -2 * PI + end->hdg;
	else if (end->hdg < -PI) end->hdg = 2 * PI + end->hdg;

	add_child(td, end);

	return td;
}

bool update_tree(node** root, node* goal)
{ 
	bool done = false;

	// Find node nearest goal
	list* nearest_list = near(*root, goal, 1);
	node* nearest_goal = nearest_list[0].node;
	free(nearest_list);

	// Find node at time interval on way to nearest_goal
	std::deque<node*> r2e_list = root_to_end(nearest_goal);
	node* comm_end = *root;
	while ((comm_end->time - (*root)->time) < t_int){
		r2e_list.pop_front();
		if (r2e_list.empty()){
			done = true;
			break;
		}
		comm_end = r2e_list[0];	
		add_to_commit(comm_end);
	}

	// Prune tree
	prune_tree(root, comm_end);
	*root = comm_end;
	(*root)->parent = NULL;

	return done;
}

disp disp_info(node* from, node* to)
{
	float L;

	float r = sqrtf(powf(to->coord[0] - from->coord[0], 2) + powf(to->coord[1] - from->coord[1], 2) + powf(to->coord[2] - from->coord[2], 2));
	float theta = atan2f(to->coord[1] - from->coord[1], to->coord[0] - from->coord[0]) - from->hdg;

	if (theta > PI) theta = -2 * PI + theta;
	else if (theta < -PI) theta = 2 * PI + theta;

	if (theta != 0) L = r*theta / sin(theta);
	else L = r;

	disp d = {L, L};

	return d;
}

bool collision(node* root)
{
	bool collision = false;
	node* from = root;
	node* to;

	while (from->child)
	{
		to = from->child;
		// Check if node is in world
		if (to->coord[0] < 0.0f || to->coord[0] > x_max || to->coord[1] < 0.0f || to->coord[1] > y_max || to->coord[2] < 0.0f || to->coord[2] > z_max)
			return collision = true;
		
		from = to;
	}

	return collision;
}

void free_tree(node** n)
{
	if (*n == NULL)
		return;

	free_tree(&((*n)->child));
	free_tree(&((*n)->next));

	free(*n);
	*n = NULL;
}

void prune_tree(node** n, node* new_root)
{
	if (*n == NULL || *n == new_root)
		return;

	prune_tree(&((*n)->child), new_root);
	prune_tree(&((*n)->next), new_root);

	free(*n);
	*n = NULL;
}

list* near(node* root, node* to, int near_type)
{
	list *list_arr = NULL;
	int sot = tree_size(root);
	list_arr = (list*)calloc(sot, sizeof(list));
	add_to_array(root, to, list_arr, 0);
	if (near_type == 0) qsort(list_arr, sot, sizeof(list), compare);
	else if (near_type == 1) qsort(list_arr, sot, sizeof(list), smart_compare);
	
	return list_arr;
}

int compare(const void* v1, const void* v2)
{
	const list *p1 = (list*)v1;
	const list *p2 = (list*)v2;
	if (p1->nearness < p2->nearness)
		return -1;
	else if (p1->nearness > p2->nearness)
		return +1;
	else
		return 0;
}

int smart_compare(const void* v1, const void* v2)
{
	const list *p1 = (list*)v1;
	const list *p2 = (list*)v2;
	if (p1->smart_nearness > p2->smart_nearness)
		return -1;
	else if (p1->smart_nearness < p2->smart_nearness)
		return +1;
	else
		return 0;
}

int tree_size(node* root)
{
	if (root == NULL) return 0;
	else{
		return (tree_size(root->child) + 1 + tree_size(root->next));
	}
}

int add_to_array(node* n, node* to, list* arr, int i)
{
	if (n == NULL)
		return i;

	arr[i].node = n;
	arr[i].nearness = disp_info(n, to).length;

	float D = sqrtf(powf(n->coord[0] - start_coord[0], 2) + powf(n->coord[1] - start_coord[1], 2) + powf(n->coord[2] - start_coord[2], 2));
	float L = n->cost;
	float D2G = disp_info(n, to).length;

	arr[i].smart_nearness = D / (L + D2G);

	i++;
	if (n->child != NULL) i = add_to_array(n->child, to, arr, i);
	if (n->next != NULL) i = add_to_array(n->next, to, arr, i);

	return i;
}

std::deque<node*> root_to_end(node* end)
{
	node* temp = end;
	std::deque<node*> r2e_list(1, temp);
	while (temp->parent != NULL){
		temp = temp->parent;
		r2e_list.push_front(temp);
	}

	return r2e_list;
}

void add_to_commit(node* n)
{
	node* new_n = (node*)malloc(sizeof(node));
	memcpy(new_n, n, sizeof(node));
	comm_vec.push_back(new_n);
}

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
				norm_temp = sqrtf(powf(towards->coord[0] - temp->coord[0], 2) + powf(towards->coord[1] - temp->coord[1], 2) + powf(towards->coord[2] - temp->coord[2], 2));
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
	if (dt > dt_max) dt = dt_max;

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

/*
vector<float> generateRange(float a, float b, float c) {
	vector<float> array;
	if (a > c) array.push_back(a);
	while (a <= c) {
		array.push_back(a);
		a += b;        
	}
	return array;
}

vector<float> linspace(float a, float b, int n) {
	vector<float> array;
	float step = (b - a) / (n - 1);

	while (a <= b) {
		array.push_back(a);
		a += step;
	}
	return array;
}
*/

/*
node* search_tree(node* n, int ID)
{
	if (n == NULL) return NULL;
	if (n->ID == ID) return n;
	node* n_found;
	if ((n_found = search_tree(n->next, ID)) != NULL) return n_found;
	return search_tree(n->child, ID);
}
*/

/*
node* compare_tree(node* n)
{
	if (n == NULL) return NULL;

	int greatest_ID = n->ID;
	node* temp = n;

	std::queue<node*> Q;
	Q.push(n);
	while (!Q.empty()){
		node* current = Q.front();
		if (current->ID > greatest_ID) temp = current;
		if (current->child != NULL) Q.push(current->child);
		if (current->next != NULL) Q.push(current->next);
		Q.pop();
	}
	return temp;
}
*/

/*
node* extend_commited_tree(node* root)
{
	node* commited_node;
	if (root == NULL)  return;

	node* temp = root;
	printf("backwards->ID: %d\n", temp->ID);
	while (temp->parent != NULL)
	{
		temp = temp->parent;
		printf("backwards->ID: %d\n", temp->ID);
	}
}
*/
