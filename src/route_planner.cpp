#include "route_planner.h"
#include <algorithm>

// Constructor method.
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // (DONE) TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// (DONE) TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// (DONE) TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    current_node->visited = true;  // Don't forget to mark the current node as visited!!!
  	// (Special thanks to Tiago D. on the page https://knowledge.udacity.com/questions/85529 for pointing this out!)
    for (auto i : current_node->neighbors) {
        i->parent = current_node;
        i->g_value = current_node->g_value + current_node->distance(*i);
        i->h_value = CalculateHValue(i);
        i->visited = true;
        open_list.push_back(i);
    }
}


// (DONE) TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    auto comp = [] (RouteModel::Node *a, RouteModel::Node *b) {
        double f1 = a->g_value + a->h_value; // f1 = g1 + h1
        double f2 = b->g_value + b->h_value; // f2 = g2 + h2
        return f1 > f2; 
    };
    sort(open_list.begin(), open_list.end(), comp);
    RouteModel::Node *lowest;
    lowest = open_list.back();
    open_list.pop_back();
    return lowest;
}


// (DONE) TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vec
    std::vector<RouteModel::Node> path_found;
    std::vector<RouteModel::Node*> temp;  // Vector of pointers to the nodes for reversing.

    RouteModel::Node *last = nullptr; 
    RouteModel::Node *iter = current_node;

    // (DONE) TODO: Implement your solution here.

    while (true) {
        if (last != nullptr) {
            distance += last->distance(*iter);
        }
        temp.push_back(iter);
        if (iter == start_node) {
            break;
        }
        last = iter;
        iter = iter->parent;
    }

    while (temp.size() > 0) {
        RouteModel::Node* curr = temp.back();
        temp.pop_back();
        path_found.push_back(*curr);
    }

    this->distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// (DONE) TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    // (DONE) TODO: Implement your solution here.

    open_list.push_back(start_node);

    while (open_list.size() > 0) {
        // Get the next node...
        current_node = NextNode();

        // Check if we're done...
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }

        // If we're not done, get the current node's neighbors...
        AddNeighbors(current_node);
    }
}