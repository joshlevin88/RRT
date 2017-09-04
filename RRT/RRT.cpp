#include<iostream>
#include<vector>
#include<queue>
#include<math.h>
#include<time.h>

#define PI 3.14159265358979323846f

using std::vector;

// Tree
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

// Displacement
struct disp{
	float length;
	float cost;
};
typedef struct disp disp;

// List for nearest sorting
struct list{
	struct node* node;
	float nearness;
};
typedef struct list list;

// Constants
const float V = 5.0f;
const float ts = 0.5f;
const float bf = 1.5f;
const float t_int = 0.5f;
const int max_tr_deg = 50;
const int max_zr = 1;
const float coll_dist = 0.2f;
const float ATA_dist = 9.0f;
const float t_td = 0.23f;
const float dt_max = 1.0f;
const int near_max = 5;
const float x_max = 50.0f;
const float y_max = 30.0f;
const float z_max = 10.0f;

// Functions
node* new_node(float, float, float, float, int, int, int, float, float, node*);
node* rand_node_coord(node*, int);
node* add_sibling(node*, node*);
node* add_child(node*, node*);
void trim_end_states(node*, node*, int, int, float);
void extend_tree(node*, node*);
node* steer_an(node*, node*);
disp disp_info(node*, node*);
bool collision(node*);
void free_tree(node*);
node* steer(node*, node*);
list* nearest(node*, node*);
int compare(const void*, const void*);
int tree_size(node*);
int add_to_array(node* n, node* to, list arr[], int i);
/*
vector<float> generateRange(float, float, float);
vector<float> linspace(float, float, int);
node* search_tree(node*, int);
node* compare_tree(node*);
void backwards(node*);
*/


int main()
{
	node* root = new_node(1.0f, 15.0f, 5.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);
	node* second = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 1.0f, 0.0f, NULL);
	trim_end_states(second, root, 0, 0, 1.0f);
	add_child(root, second);

	bool path_found = false;
	bool done = false;
	int iter = 0;
	node* commited_root = (node*)malloc(sizeof(node));
	commited_root = root;
	int gn = 1;
	node* goal = new_node(45.0f, 25.0f, 5.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);

	node* temp;
	node* prim_root;
	node* rand_node = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);
	while (!done){

		time_t start, end;
		double elapsed;
		bool done_iter = false;

		start = time(NULL);
		while (!done_iter){

			end = time(NULL);
			elapsed = difftime(end, start);
			if (elapsed >= t_int){
				done_iter = true;
				break;
			}

			rand_node_coord(rand_node, iter);

			temp = root;
			while (temp->child) temp = temp->child;

			prim_root = steer_an(temp, rand_node);
			if (collision(prim_root) == true) free_tree(prim_root);
			else {
				add_child(temp, prim_root);
			}

			iter++;
		}

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
node* rand_node_coord(node* rand_node, int iter)
{
	int bias_freq = 3;
	float goal_coord[3] = {50.0f, 30.0f, 10.0f};

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
node* add_child(node* n, node* new_n)
{
	if (n == NULL) return NULL;

	if (n->child) return add_sibling(n->child, new_n);

	else{
		new_n->parent = n;
		return n->child = new_n;
	}
}

// Add node as sibling if parent node already has a child
node* add_sibling(node* n, node* new_n)
{
	if (n == NULL) return NULL;

	while (n->next) n = n->next;

	new_n->parent = n->parent;
	return n->next = new_n;
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

void extend_tree(node* root, node* rand_node)
{

}

node* steer_an(node* from, node* towards)
{
	// Root of primtive is time-delayed node
	node* td = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 1, from->time + t_td, 0.0f, NULL);
	trim_end_states(td, from, from->tr_deg, from->zr, t_td);
	disp d = disp_info(from, td);
	td->cost = d.cost;

	float dt;
	int tr_deg;
	int zr;
	float dt_i = ts;

	// Find best trim primitive and time spent along it
	float r_G = sqrtf(powf(towards->coord[0] - td->coord[0], 2) + powf(towards->coord[1] - td->coord[1], 2) + powf(towards->coord[2] - td->coord[2], 2));
	float theta_G = atan2(towards->coord[1] - td->coord[1], towards->coord[0] - td->coord[0]) - td->hdg;
	if (theta_G > PI) theta_G = -2 * PI + theta_G;
	else if (theta_G < -PI) theta_G = 2 * PI + theta_G;
	float r_c = sqrtf(powf(towards->coord[0] - td->coord[0], 2) + powf(towards->coord[1] - td->coord[1], 2)) / (2 * sin(theta_G));
	float L;
	if (theta_G != 0) L = r_G*theta_G / sin(theta_G); 
	else L = r_G;
	dt = L / V;
	tr_deg = int(V / r_c * 180.0f / PI);
	tr_deg = ((tr_deg + 10 / 2) / 10) * 10;
	if (dt > dt_max) dt = dt_max;
	zr = int((towards->coord[2] - td->coord[2]) / dt);

	if (tr_deg > max_tr_deg) tr_deg = max_tr_deg;
	else if (tr_deg < -max_tr_deg) tr_deg = -max_tr_deg;
	if (zr > max_zr) zr = max_zr;
	else if (zr < -max_zr) zr = -max_zr;


	td->tr_deg = tr_deg;
	td->zr = zr;

	node* last = td;
	dt_i = ts;
	if (ts > dt) dt_i = dt;

	// Add intermediate nodes along trim primitive to tree
	node* temp;
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

void free_tree(node* n)
{
	if (n == NULL)
		return;

	free_tree(n->child);
	free_tree(n->next);

	free(n);
}


list* nearest(node* root, node* to)
{
	list *list_arr = NULL;
	int sot = tree_size(root);
	list_arr = (list*)calloc(sot, sizeof(list));
	add_to_array(root, to, list_arr, 0);
	qsort(list_arr, sot, sizeof(list), compare);
	return list_arr;

	/*
	for (int i = 0; i<sot; i++)
	{
		printf("arr[%d]: %p, %f\n", i, arr[i].node, arr[i].nearness);
	}
	qsort(arr, sot, sizeof(list), compare);
	for (int i = 0; i<sot; i++)
	{
		printf("arr[%d]: %p, %f\n", i, arr[i].node, arr[i].nearness);
	}
	*/
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

int tree_size(node* root)
{
	if (root == NULL) return 0;
	else{
		return (tree_size(root->child) + 1 + tree_size(root->next));
	}
}

int add_to_array(node* n, node* to, list arr[], int i)
{
	if (n == NULL)
		return i;

	arr[i].node = n;
	arr[i].nearness = disp_info(n, to).length;
	i++;
	if (n->child != NULL) i = add_to_array(n->child, to, arr, i);
	if (n->next != NULL) i = add_to_array(n->next, to, arr, i);

	return i;
}

node* steer(node* from, node* towards)
{
	// Root of primtive is time-delayed node
	node* td = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 1, from->time + t_td, 0.0f, NULL);
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