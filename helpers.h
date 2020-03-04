#pragma once

#ifndef HELPERS_2019
#define HELPERS_2019

#include <vector>
#include <string>

#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

std::vector<std::string> load_test(const std::string& path)
{
  std::vector<std::string> result;

  for (const auto& f : fs::directory_iterator(path))
  {
    result.push_back(f.path().string());
  }

  return result;
}

#endif //HELPERS_2019