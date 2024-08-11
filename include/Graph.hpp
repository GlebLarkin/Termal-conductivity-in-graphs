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
  static Graph& getInstance(const std::string& filename);

  // working with temperature block
  double_t findAvarageTemperature() const;
  double_t maxTemperatureDifference() const;
  bool isTemperatureStabilized() const;

  bool showGraphData() const;

private:
  Graph(const std::string& filename);

  Graph(const Graph&) = delete;
  Graph& operator=(const Graph&) = delete;

  //just for the constructor
  bool parseJson(const nlohmann::json& json_data);

  std::unordered_map<uint32_t, Node> adjacency_list_;
};


#endif
