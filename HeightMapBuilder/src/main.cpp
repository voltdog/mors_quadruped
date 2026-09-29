#include "HeightMapBuilder/HeightMapBuilder.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

std::string ResolveConfigPath(int argc, char** argv)
{
    if (argc > 1)
    {
        return argv[1];
    }

    std::vector<std::string> candidates;

    const char* config_dir = std::getenv("CONFIGPATH");
    if (config_dir != nullptr && config_dir[0] != '\0')
    {
        candidates.emplace_back(std::string(config_dir) + "/heightmap_builder.yaml");
    }
    candidates.emplace_back("config/heightmap_builder.yaml");
    candidates.emplace_back("../config/heightmap_builder.yaml");
    candidates.emplace_back("../../config/heightmap_builder.yaml");

    for (const auto& path : candidates)
    {
        if (std::filesystem::exists(path))
        {
            return path;
        }
    }

    if (!candidates.empty())
    {
        return candidates.front();
    }
    return "heightmap_builder.yaml";
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        const std::string config_path = ResolveConfigPath(argc, argv);
        hmb::HeightMapBuilderNode node(config_path);
        return node.Run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[HeightMapBuilder] fatal: " << e.what() << std::endl;
        return 1;
    }
}
