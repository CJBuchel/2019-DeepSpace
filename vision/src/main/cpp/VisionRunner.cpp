#include "VisionRunner.h"
#include <chrono>
#include <iostream>

// Handles threading
void VisionRunner::Run(Runnable &run) {
  std::cout << "VisionRunner Started" << std::endl;
  workers.push_back(std::thread([&]() {
    run.Init();
    while (true) {
      run.Periodic();
      std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / 120.0));
    }
  }));
  std::cout << "VisionRunner Ended" << std::endl;
}