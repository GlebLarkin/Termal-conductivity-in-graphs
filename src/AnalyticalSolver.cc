#include "AnalyticalSolver.hpp"

std::shared_ptr<AnalyticalSolver> AnalyticalSolver::instance_ = nullptr;
std::mutex AnalyticalSolver::mutex_;

std::shared_ptr<AnalyticalSolver> AnalyticalSolver::getInstance()
{
  std::lock_guard<std::mutex> lock(mutex_); 
  if (!instance_)
  {
    instance_ = std::shared_ptr<AnalyticalSolver>(new AnalyticalSolver(Graph::getInstance()));
  }
  return instance_;
}

AnalyticalSolver::AnalyticalSolver(std::shared_ptr<Graph> & graph_ptr)
  : Solver(graph_ptr), laplace_matrix_(std::move(createLaplaceMatrix())) 
  {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver = computeEigenvaluesAndEigenvectors(laplace_matrix_);
    Eigen::VectorXd eigenvalues = solver.eigenvalues(); 
    Eigen::MatrixXd eigenvectors = solver.eigenvectors();
  }


bool AnalyticalSolver::solve(const double_t measurment_time, const double_t dt) const
{
  AnalyticalSolver::computeEigenvaluesAndEigenvectors(laplace_matrix_);
  for (auto current_time = 0.0; current_time < measurment_time; current_time += dt)
  {
    auto current_temp = AnalyticalSolver::computeTemperatures(current_time);
    AnalyticalSolver::saveJsonData(current_temp, current_time);
  }
}

//this func turns keys into indexes for laplace matrix being correct (indexes < matrix size)
std::unordered_map<uint32_t, uint32_t> keyToIndex(const std::shared_ptr<const std::unordered_map<uint32_t, Node>> & map)
{
  std::unordered_map<uint32_t, uint32_t> index_map;
  auto index = 0;

  for (const auto & [key, _] : *map) index_map.emplace(key, index++);

  return index_map;
}

Eigen::MatrixXd AnalyticalSolver::createLaplaceMatrix() 
{
  const auto adjacency_list_ptr_ = graph_ptr_->getAdjacencyListPtr();
  const auto n = adjacency_list_ptr_->size();
  Eigen::MatrixXd laplace_matrix = Eigen::MatrixXd::Zero(n, n);

  auto index_map = keyToIndex(adjacency_list_ptr_);

  for (const auto& [key, node] : *adjacency_list_ptr_) 
  {
    auto i = index_map[key];

    laplace_matrix(i, i) = node.connected_edges.size();

    for (auto neighbor_key : node.connected_edges)
    {
      auto neighbor = index_map[neighbor_key]; 
      if (neighbor == i) continue;
      laplace_matrix(i, neighbor) = -1;
    }
  }

  return laplace_matrix;
}
 

Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> AnalyticalSolver::computeEigenvaluesAndEigenvectors(const Eigen::MatrixXd& laplace_matrix) const
{
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(laplace_matrix_);
  return solver;
}


std::vector<double_t> AnalyticalSolver::getTemperatures() const
{
  const auto adjacency_list_ptr = graph_ptr_->getAdjacencyListPtr();

  std::vector<double_t> temp;
  temp.reserve(adjacency_list_ptr->size());
  for (const auto & [key, node] : *(adjacency_list_ptr))
  {
    temp.push_back(node.temperature);
  }

  return temp;
}


std::vector<double_t> AnalyticalSolver::computeTemperatures(const double_t current_time) const
{
  auto initial_temperatures = getTemperatures();
  auto temperatures = initial_temperatures;
  auto n = eigenvalues.size();

  for (auto i = 0; i < n; ++i) 
  {
    auto temperature = 0.0;

    for (auto j = 0; j < n; ++j) 
    {
      auto decay = std::exp(-eigenvalues[j] * current_time);
      temperature += decay * eigenvectors(i, j) * initial_temperatures[j];
    }

    temperatures[i] = temperature;
  }

  return temperatures; 
}


bool AnalyticalSolver::saveJsonData(std::vector<double_t> &current_temperature, double_t current_time) const
{
  static bool is_first_call = 1;

  std::string file_path = "../data/analytical_output.json";

  nlohmann::json temperature_data;
  temperature_data["time"] = current_time;
  temperature_data["temperatures"] = current_temperature;

  //deletes data of previous program sessions
  if(is_first_call)
  {
    std::ofstream outfile(file_path, std::ofstream::trunc);
      if (!outfile.is_open()) 
      {
        std::cerr << "Error opening the analytical solver output file for writing." << std::endl;
        return 0;
      }

      outfile << nlohmann::json::array();
      outfile.close();

      is_first_call = 0;
  }

  //opens file for writing and writes
    std::ofstream outfile(file_path, std::ofstream::app);
    if (!outfile.is_open()) 
    {
      std::cerr << "Error opening the analytical solver output file for appending" << std::endl;
      return false;
    }

    try 
    {
      if (outfile.tellp() == 0) outfile << nlohmann::json::array();

      outfile.seekp(-1, std::ios_base::end);
      if (outfile.tellp() != 0) outfile << ",";

      outfile << temperature_data;

      outfile.close();
    } 
    catch (...) 
    {
      std::cerr << "Error writing to the analytical solver output file." << std::endl;
      return 0;
    }

  return 1;
}

bool AnalyticalSolver::solve(double_t measurment_time, double_t dt) const
{
  for (auto t = 0; t < measurment_time; t += dt)
  {
    std::vector<double_t> current_temp = computeTemperatures(t);
    if(!saveJsonData(current_temp, t)) return 0;
  }
  return 1;
}