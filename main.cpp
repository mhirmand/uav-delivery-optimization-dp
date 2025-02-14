#include "delivery_uav.h"
#include <string>
#include <iostream>

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <input_file> <output_file>\n";
    return 1;
  }

  const std::string input_file = argv[1];
  const std::string output_file = argv[2];

  DeliveryUAV uav = DeliveryUAV();
  return uav.solveCase("input.txt", "output.txt");
}