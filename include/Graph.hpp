#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "MyInclude.hpp"


struct Node
{
  uint32_t temperature;
  std::vector<int> connected_edges;
};

class Graph
{
public:
  static std::shared_ptr<Graph> & getInstance(const std::string& filename = "../data/input.json");

  // working with temperature block
  double_t findAvarageTemperature() const;
  double_t maxTemperatureDifference() const;
  bool isTemperatureStabilized() const;

  bool showGraphData() const;

  std::shared_ptr<const std::unordered_map<uint32_t, Node>> getAdjacencyListPtr() const;

private:
  Graph(const std::string& filename);

  Graph(const Graph&) = delete;
  Graph& operator=(const Graph&) = delete;

  //just for the constructor
  bool parseJson(const nlohmann::json& json_data);

  std::unordered_map<uint32_t, Node> adjacency_list_;

  static std::shared_ptr<Graph> instance_;
};


#endif
