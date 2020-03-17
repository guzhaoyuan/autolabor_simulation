//
// Created by gzy on 3/17/20.
//

#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>
#include <vector>

int main(int argc, char **argv) {
  std::random_device rd{};
  std::mt19937 gen{rd()};

  const double mean = 5;
  const double stddev = 2;
  const double variance = std::pow(stddev,2);

  // values near the mean are the most likely
  // standard deviation affects the dispersion of generated values from the mean
  std::normal_distribution<> d{mean,stddev};

  const int number = 100000;
  int count = 0;
  double sum = 0;
  std::map<int, int> hist{};
  std::vector<double> all(number);
  for(int n=0; n<number; ++n) {
    double rand = d(gen);
    ++hist[std::round(rand)];
    if (rand >= mean-stddev && rand <= mean+stddev)
      count ++;
    sum += rand;
    all[n] = rand;
  }
  double average = sum / number;

  double calculated_variance = 0;
  for (auto rand : all)
    calculated_variance += std::pow(rand-average,2);
  calculated_variance /= number;

  for(auto p : hist) {
    std::cout << std::setw(2)
              << p.first << ' ' << std::string(p.second/200, '*') << '\n';
  }
  std::cout << "data sit in 1 sigma: " << (double)count/number << std::endl;
  std::cout << "calculated variance: " << calculated_variance << std::endl;

  return 0;
}
