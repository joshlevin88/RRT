#include<iostream>
#include<vector>
using namespace std; 

struct Node{
	int ID;
	std::vector<float> coord;
	struct Node* left;
	struct Node* right;
};

int main(){
	cout << "Hi!";
}