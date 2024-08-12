#include "Graph.hpp"

double_t Graph::findAvarageTemperature() const
{
  auto total_temp = 0;

  for (const auto & [key, node] : adjacency_list_) total_temp += node.temperature;
  return static_cast<double_t> (total_temp / adjacency_list_.size());
}

double_t Graph::maxTemperatureDifference() const
{
  auto max_temp_dif = 0;
  auto avarage_temp = findAvarageTemperature();

  for (const auto & [key, node] : adjacency_list_)
  {
    auto delta = node.temperature - avarage_temp;
    if (delta > max_temp_dif) max_temp_dif = delta;
  }

  return max_temp_dif;
}

bool Graph::isTemperatureStabilized() const
{
  return (maxTemperatureDifference() < findAvarageTemperature() * 0.01); 
}

bool Graph::showGraphData() const
{
  for (const auto & [key, node] : adjacency_list_) 
  {
    std::cout << "Node: " << key << "\n";
      std::cout << "  Temperature: " << node.temperature << "\n";
      std::cout << "  Connected Edges: ";
        
      if (node.connected_edges.empty()) std::cout << "None"; 
      else 
      {
        for (auto i = 0; i < node.connected_edges.size(); ++i) 
        {
          std::cout << node.connected_edges[i];
          if (i != node.connected_edges.size() - 1) std::cout << ", ";
        }
      }

    std::cout << "\n";
  }
}

std::shared_ptr<Graph> Graph::instance_ = nullptr;

std::shared_ptr<Graph> & Graph::getInstance(const std::string & filename = "../data/input.json")
{
  if (!instance_)
  {
    instance_ = std::shared_ptr<Graph>(new Graph(filename));
    return instance_;
  }
}

Graph::Graph(const std::string & filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file");
    }

    nlohmann::json json_data;
    file >> json_data;

    parseJson(json_data);
}

bool Graph::parseJson(const nlohmann::json& json_data)
{
    for (const auto& item : json_data.items()) {
        uint32_t node_id = std::stoi(item.key());
        Node node;
        node.temperature = item.value()["temperature"];
        node.connected_edges = item.value()["connected_edges"].get<std::vector<int>>();

        adjacency_list_[node_id] = node;
    }
}

std::shared_ptr<const std::unordered_map<uint32_t, Node>> Graph::getAdjacencyListPtr() const
{
  return std::make_shared<const std::unordered_map<uint32_t, Node>>(adjacency_list_);
}