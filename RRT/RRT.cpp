#include<iostream>
#include<vector>
#include<queue>
#include<math.h>
#include<time.h>
#define PI 3.14159265358979323846f

// Tree data structure
struct node{
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
};
typedef struct node node;

// Constants
const float V = 5.0f;
const float t_iter = 0.5f;
const float bf = 1.5f;
const float t_int = 0.5f;
const int max_tr_deg = 50;
const int maz_zr = 1;
const float coll_dist = 0.2f;
const float ATA_dist = 9.0f;
const float td = 0.23f;
const float dt_max = 1.0f;
const int near_max = 5;

// Functions
node* new_node(float, float, float, float, int, int, int, float, float, node*);
node* rand_node(int);
node* add_sibling(node*, float, float, float, float, int, int, int, float, float);
node* add_child(node*, float, float, float, float, int, int, int, float, float);
void trim_end_states(node*, node*, int, int, float);
/*
node* search_tree(node*, int);
node* compare_tree(node*);
void backwards(node*);
*/


int main()
{
	node* root = new_node(0, 0, 0, 0, 0, 0, 0, 0, 0, NULL);
	node* second = add_child(root, 10, 0, 0, 0, 0, 0, 0, 2, 0);
	printf("second - coord: [%.1f, %.1f, %.1f] - heading: %.1f \n", second->coord[0], second->coord[1], second->coord[2], second->hdg*180/PI);
	trim_end_states(second, root, -40, 1, 10);
	printf("second - coord: [%.1f, %.1f, %.1f] - heading: %.1f \n", second->coord[0], second->coord[1], second->coord[2], second->hdg*180/PI);

	bool path_found = false;
	bool done = false;
	int iter = 0;
	node* commited_root = (node*)malloc(sizeof(node));
	commited_root = root;
	int gn = 1;
	node* goal = new_node(45, 25, 5, 0, 0, 0, 0, 0, 0, NULL);

	while (!done){

		time_t start, end;
		double elapsed;
		start = time(NULL);
		bool done_iter = false;

		while (!done_iter){
			end = time(NULL);
			elapsed = difftime(end, start);
			if (elapsed >= t_iter)
				done_iter = true;
		}
		printf("Interval finished!");
	}
}

// Allocate dynamic memory and initialize new node
node* new_node(float coord_x, float coord_y, float coord_z, float hdg, int tr_deg, int zr, int type, float time, float cost, node* parent)
{
	node* new_node = (node*) malloc(sizeof(node));

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
node* rand_node(int iter)
{
	node* rand_node = new_node(0, 0, 0, 0, 0, 0, 0, 0, 0, NULL);
	int bias_freq = 3;
	float goal_coord[3] = {50.0f, 30.0f, 10.0f};
	float x_max = 50.0f;
	float y_max = 30.0f;
	float z_max = 10.0f;

	// Every bias_freq iterations, return goal node
	if (iter % bias_freq == 0) {
		rand_node->coord[0] = goal_coord[0];
		rand_node->coord[1] = goal_coord[1];
		rand_node->coord[2] = goal_coord[2];
	}
	else{
		rand_node->coord[0] = ((float)rand() / (float)(RAND_MAX)) * x_max;
		rand_node->coord[1] = ((float)rand() / (float)(RAND_MAX)) * y_max;
		rand_node->coord[2] = ((float)rand() / (float)(RAND_MAX)) * z_max;
	}

	return rand_node;
}

// Add node to tree
node* add_child(node* n, float coord_x, float coord_y, float coord_z, float hdg, int tr_deg, int zr, int type, float time, float cost)
{
	if (n == NULL) return NULL;
	if (n->child) return add_sibling(n->child, coord_x, coord_y, coord_z, hdg, tr_deg, zr, type, time, cost);
	else return (n->child = new_node(coord_x, coord_y, coord_z, hdg, tr_deg, zr, type, time, cost, n));
}

// Add node as sibling if parent node already has a child
node* add_sibling(node* n, float coord_x, float coord_y, float coord_z, float hdg, int tr_deg, int zr, int type, float time, float cost)
{
	if (n == NULL) return NULL;

	while (n->next) n = n->next;

	return (n->next = new_node(coord_x, coord_y, coord_z, hdg, tr_deg, zr, type, time, cost, n->parent));
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
void backwards(node* n)
{
	if (n == NULL)  return;

	node* temp = n;
	printf("backwards->ID: %d\n", temp->ID);
	while (temp->parent != NULL)
	{
		temp = temp->parent;
		printf("backwards->ID: %d\n", temp->ID);
	}
}
*/