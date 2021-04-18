#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y); 
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float H = node->distance(*end_node);
    return H;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    current_node->visited = true;
    for (auto *AllNode : current_node->neighbors)
    {
        AllNode->parent = current_node;
        AllNode->h_value = CalculateHValue(AllNode);
        AllNode->g_value = current_node->g_value + AllNode->distance(*current_node);
        open_list.push_back(AllNode);
        AllNode->visited = true;
        /* code */
    }
    
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    //MEMO: sorting the vector 2D comparing the sum of h and g value
    std::sort(open_list.begin(), open_list.end(), [](const auto &before, const auto &after)
    {return before->h_value + before->g_value < after->h_value + after->g_value;});

    int sizeOfList = open_list.size()-1;
    RouteModel::Node *lowestSumNode = open_list[sizeOfList];
    open_list.pop_back();
    return lowestSumNode;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    //MEMO: with the TODO i must put all the series of node founded before, but starting from the last (end_node or goal)
    //      and puttin these into path_found.

    //MEMO: putting in path_found the "end_node"
    path_found.push_back(*current_node);

    //MEMO: this while is need because when the while will reach the "start" will not have a parent. ad we will done for the path
    while(current_node->parent != nullptr){
        //Adding the distance between the current node and the parent, 
        if(current_node->parent != nullptr && current_node->parent != current_node){
            distance = distance + (current_node->distance(*current_node->parent));
        }
        
        //Changing the current node with it's parent, to proceed with the while
        current_node = current_node->parent;
        //Adding into path_found the next node
        path_found.push_back(*current_node);
    }

    //initializating a vector<RouteModel::Node> to put inside the right order from the "start_node" to the "end_node"
    std::vector<RouteModel::Node> CorrectOrder;
    for(int counter = path_found.size()-1 ; counter != -1 ; counter--){
        CorrectOrder.push_back(path_found[counter]);
    }
    path_found = CorrectOrder;

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    open_list.push_back(start_node);
    start_node->visited = true;
    while(open_list.size() != 0){
        current_node = NextNode();
        if(current_node->distance(*end_node) == 0){
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        std::cout << " current node: " << current_node << "\n";
        std::cout << " current node distance from end: " << current_node->distance(*end_node) << "\n";
        current_node->visited = true;
        AddNeighbors(current_node);
    }
}