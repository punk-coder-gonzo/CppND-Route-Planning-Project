#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if (!is)
        return std::nullopt;

    auto size = is.tellg();
    std::vector<std::byte> contents(size);

    is.seekg(0);
    is.read((char *)contents.data(), size);

    if (contents.empty())
        return std::nullopt;
    return std::move(contents);
}

static void print_usage(std::string progname)
{
    std::cerr << "To specify a map file, start and end coordinates use the following format:\n"
              << "Usage: " << progname << " <options>\n"
              << "Options:\n"
              << "  -h,--help\tShow this help message\n"
              << "  -f,--file\tinput map filename [default: ../map.osm]\n"
              << "  -r,--route\tfour floating point values start_x, start_y, end_x, end_y [default: 10 10 90 90]\n"
              << "\n"
              << "Example:" << progname << " -f ../map.osm -r 10 10 90 90\n"
              << std::endl;
}

int main(int argc, const char **argv)
{    
    float start_x = 10, start_y = 10, end_x = 90, end_y = 90;
    std::string osm_data_file = "/home/witter/work/udacity-cpp/CppND-Route-Planning-Project/map.osm"; // default

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help"))
        {
            print_usage(argv[0]);
            return 0;
        }

        if ((arg == "-f") || (arg == "--file"))
        {
            if ((i + 1) < argc)
                osm_data_file = argv[++i];
            else
            {
                std::cerr << "-f,--file option requires one argument." << std::endl;
                return 1;
            }
        }

        if ((arg == "-r") || (arg == "--route"))
        {
            if ((i + 4) < argc)
            {
                start_x = atof(argv[++i]);
                start_y = atof(argv[++i]);
                end_x = atof(argv[++i]);
                end_y = atof(argv[++i]);
            }
            else
            {
                std::cerr << "-r,--route option requires four arguments." << std::endl;
                return 1;
            }
        }
    }

    std::vector<std::byte> osm_data;

    if (osm_data.empty() && !osm_data_file.empty())
    {
        std::cout << "Reading OpenStreetMap data from the following file: " << osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if (!data)
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface &surface) {
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface &surface) {
        render.Display(surface);
    });
    display.begin_show();
}
