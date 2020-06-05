#include <eigen3/Eigen/Core>
#include <functional>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>

using namespace Eigen;

// Create springs
// Create masses
// Connectivity
// springs know masses
// loop on springs


struct Mass {
  float m;
  Vector2f position, velocity, forcesApplied;
  bool isFixed;
  Mass(float x, float y, float m) : m(m), position(Vector2f(x, y)), velocity(0,0), isFixed(false) {}
  Mass(float x, float y) : m(0.f), position(Vector2f(x, y)), velocity(0,0), isFixed(true) {}
  friend std::ostream& operator<<(std::ostream& os, const Mass& m) {
    os << m.position(0) << ' ' << m.position(1) << ' '
       << m.velocity(0) << ' ' << m.velocity(1) << '\n';
    return os;
  }
};

struct Spring {
  float k, l;
  Mass *mass1, *mass2;
  inline Vector2f force() const {
    const Vector2f& x1 = mass1->position;
    const Vector2f& x2 = mass2->position;
    const Vector2f direction = 1./(x2-x1).norm() * (x2-x1);
    return k*((x2-x1).norm()-l)*direction;
  }
};


struct System {
  std::vector<Mass> masses;
  std::vector<Spring> springs;
  using Connectivity = std::map<std::size_t, std::pair<std::size_t, std::size_t> >;
  System(const std::vector<Mass>& masses_,
         const std::vector<Spring>& springs_,
         Connectivity connectivity) :
    masses(std::move(masses_)),
    springs(std::move(springs_))
  {
    for(const auto& c : connectivity) {
      springs[c.first].mass1 = &masses[c.second.first];
      springs[c.first].mass2 = &masses[c.second.second];
    }
  }

  void forcesZero() {
    for(auto & m : masses)
      m.forcesApplied = Vector2f::Zero();
  }

  void updateForces() {
    for(auto &s : springs) {
      s.mass1->forcesApplied += s.force();
      s.mass2->forcesApplied -= s.force();
    }
  }

  void applyForces(float dt) {
    for(auto& m : masses) {
      if (!m.isFixed) {
        m.velocity += (dt/m.m) * m.forcesApplied;
        m.position += dt * m.velocity;
      }
    }
  }

  void process(float dt) {
    forcesZero();
    updateForces();
    applyForces(dt);
  }
  
  void displayMass(size_t n, std::ostream& os = std::cout) {
    os << masses[n];
  }
};

struct Test {
  std::string description;
  std::function<void(void)> runFunction;
  void execute() const {
    std::cout << description << "...";
    runFunction();
    std::cout << "done" << std::endl;
  }
};

static const std::vector<Test> tests = {
                     {
                      .description = "One spring, one moving mass",
                      .runFunction = []{
                                       std::vector<Mass> masses = {
                                                                   Mass(0, 0),
                                                                   Mass(0, -3, 3)
                                       };

                                       std::vector<Spring> springs = {
                                                                      {.k=3, .l=2}
                                       };
                                         
                                       // Spring 0 connects masses 0 and 1
                                       System::Connectivity connectivity = {
                                                                    {0, {0, 1}}
                                       };

                                       System s(masses, springs, connectivity);

                                       const float dt = .1;
                                       const std::size_t timesteps = 1000;

                                       std::ofstream f("1m1s.dat");

                                       for (size_t ii = 0; ii < timesteps; ii++) {
                                         s.displayMass(1, f);
                                         s.process(dt);
                                       }
                                     }
                     },
                     {
                      .description = "One mass, four springs attached",
                      .runFunction = []{
                                       std::vector<Mass> masses = {
                                                                   Mass(0, 0),
                                                                   Mass(1, 0),
                                                                   Mass(0, 1),
                                                                   Mass(1, 1),
                                                                   Mass(.2, .6, 1)
                                       };

                                       std::vector<Spring> springs;
                                       for (std::size_t ii = 0; ii < 4; ii++)
                                         springs.push_back({.k=2, .l=2});

                                         
                                       System::Connectivity connectivity;
                                       for (std::size_t ii = 0; ii < 4; ii++)
                                         connectivity[ii] = std::make_pair(ii, 4);

                                       System s(masses, springs, connectivity);

                                       const float dt = .01;
                                       const std::size_t timesteps = 10000;

                                       std::ofstream f("1m4s.dat");

                                       for (size_t ii = 0; ii < timesteps; ii++) {
                                         s.displayMass(4, f);
                                         s.process(dt);
                                       }
                                     }
                     }
};


int main() {
  for(auto& test : tests)
    test.execute();
}
