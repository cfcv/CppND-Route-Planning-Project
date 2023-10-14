#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);   
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors){
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);

        neighbor->visited = true;
        open_list.emplace_back(neighbor);
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    // observation: I tried to overload the operator< in the class RouteModel::Node but it didn't sort 
    // correctly the list. It only worked when providing the function directly to the std::sort function
    // even if they are exactly the same... Operator function can be checked at route_model.cpp and route_model.h
	std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b)
	{
		return (a->h_value + a->g_value) < (b->h_value + b->g_value);
	});
    // std::sort(open_list.begin(), open_list.end());

	RouteModel::Node* closest_node = open_list.front();
	open_list.erase(open_list.begin());
	return closest_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    path_found.insert(path_found.begin(), *current_node);
    
    RouteModel::Node* current_parent = nullptr;

    while (current_node != start_node){
        current_parent = current_node->parent;
        distance += current_node->distance(*current_parent);


        current_node = current_parent;
        path_found.insert(path_found.begin(), *current_node);
    }
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    start_node->visited = true;
	open_list.emplace_back(start_node);

    while (!open_list.empty()){
        current_node = NextNode();

        if(current_node == end_node){
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        AddNeighbors(current_node);
        
    }
}