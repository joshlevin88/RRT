#include<iostream>
#include<vector>
#include<queue>

struct node{
	float coord[3];
	float hdg;
	int tr;
	int zr;
	int type;
	float time;
	float cost;
	struct node* next;
	struct node* child;
	struct node* parent;
};

typedef struct node node;

node* new_node(float, float, float, float, int, int, int, float, float, node*);
node* new_rand_node(int);
node* add_sibling(node*, float, float, float, float, int, int, int, float, float);
node* add_child(node*, float, float, float, float, int, int, int, float, float);

/*
node* search_tree(node*, int);
node* compare_tree(node*);
void backwards(node*);
*/


int main()
{
	node* root = new_node(0, 0, 0, 0, 0, 0, 0, 0, 0, NULL);
	add_child(root, 10, 0, 0, 0, 0, 0, 0, 2, 0);

	/*
	int i;
	node* root = new_node(NULL);
	node* temp = root;
	for (i = 1; i <= 12; i++){
		if (i % 2 == 0) temp = add_child(temp);
		else temp = add_sibling(temp);
	}
	*/
}

node* new_node(float coord_x, float coord_y, float coord_z, float hdg, int tr, int zr, int type, float time, float cost, node* parent)
{
	node* new_node = (node*) malloc(sizeof(node));
	if (new_node) {
		new_node->coord[0] = coord_x;
		new_node->coord[1] = coord_y;
		new_node->coord[2] = coord_z;
		new_node->hdg = hdg;
		new_node->tr = tr;
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

node* new_rand_node(int iter)
{
	node* new_rand_node = new_node(0, 0, 0, 0, 0, 0, 0, 0, 0, NULL);
	int bias_freq = 3;
	float goal_coord[3] = {50.0, 30.0, 10.0};
	float x_max = 50.0;
	float y_max = 30.0;
	float z_max = 10.0;

	if (iter % bias_freq == 0) {
		new_rand_node->coord[0] = goal_coord[0];
		new_rand_node->coord[1] = goal_coord[1];
		new_rand_node->coord[2] = goal_coord[2];
	}
	else{
		new_rand_node->coord[0] = ((float)rand() / (float)(RAND_MAX)) * x_max;
		new_rand_node->coord[1] = ((float)rand() / (float)(RAND_MAX)) * y_max;
		new_rand_node->coord[2] = ((float)rand() / (float)(RAND_MAX)) * z_max;
	}

	return new_rand_node;
}

node* add_sibling(node* n, float coord_x, float coord_y, float coord_z, float hdg, int tr, int zr, int type, float time, float cost)
{
	if (n == NULL) return NULL;

	while (n->next) n = n->next;

	return (n->next = new_node(coord_x, coord_y, coord_z, hdg, tr, zr, type, time, cost, n->parent));
}

node* add_child(node* n, float coord_x, float coord_y, float coord_z, float hdg, int tr, int zr, int type, float time, float cost)
{
	if (n == NULL) return NULL;
	if (n->child) return add_sibling(n->child, coord_x, coord_y, coord_z, hdg, tr, zr, type, time, cost);
	else return (n->child = new_node(coord_x, coord_y, coord_z, hdg, tr, zr, type, time, cost, n));
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