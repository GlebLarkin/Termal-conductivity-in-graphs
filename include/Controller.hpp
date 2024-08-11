#fndef CONTROLLER_HPP
#define CONTROLLER_HPP


class Controller
//Singleton
{
public:
  static Controller& getInstance();

  bool numericalSolve() const;
  bool analyticalSolve() const;

  bool compareMethods() const;

  bool showTemperatures() const;
  bool plotGraph() const;

private:
  Controller();
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
};


#endif
