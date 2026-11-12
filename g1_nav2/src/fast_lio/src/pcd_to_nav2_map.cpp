#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

struct Options
{
    std::string input_path;
    fs::path output_dir;
    double resolution = 0.05;
    double z_min = 0.05;
    double z_max = 2.0;
    int min_component_cells = 4;
};

void print_usage(const char *program)
{
    std::cerr << "Usage: " << program
              << " INPUT.pcd OUTPUT_DIR [resolution] [z_min] [z_max] [min_component_cells]\n"
              << "Example: " << program
              << " src/map/g1_fastlio.pcd src/map 0.05 0.05 2.0 4\n";
}

bool parse_options(int argc, char **argv, Options &options)
{
    if (argc < 3 || argc > 7)
    {
        print_usage(argv[0]);
        return false;
    }

    options.input_path = argv[1];
    options.output_dir = argv[2];
    try
    {
        if (argc >= 4) options.resolution = std::stod(argv[3]);
        if (argc >= 5) options.z_min = std::stod(argv[4]);
        if (argc >= 6) options.z_max = std::stod(argv[5]);
        if (argc >= 7) options.min_component_cells = std::stoi(argv[6]);
    }
    catch (const std::exception &error)
    {
        std::cerr << "Invalid numeric argument: " << error.what() << "\n";
        return false;
    }

    if (!(options.resolution > 0.0) || options.z_min >= options.z_max ||
        options.min_component_cells < 1)
    {
        std::cerr << "resolution must be > 0, z_min < z_max, and "
                  << "min_component_cells >= 1\n";
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    Options options;
    if (!parse_options(argc, argv, options)) return 2;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(options.input_path, cloud) != 0)
    {
        std::cerr << "Failed to load PCD: " << options.input_path << "\n";
        return 1;
    }

    std::vector<const pcl::PointXYZ *> points;
    points.reserve(cloud.size());
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto &point : cloud.points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z) || point.z < options.z_min ||
            point.z > options.z_max)
        {
            continue;
        }
        points.push_back(&point);
        min_x = std::min(min_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_x = std::max(max_x, static_cast<double>(point.x));
        max_y = std::max(max_y, static_cast<double>(point.y));
    }

    if (points.empty())
    {
        std::cerr << "No points remain after Z filtering ["
                  << options.z_min << ", " << options.z_max << "]\n";
        return 1;
    }

    const double origin_x = std::floor(min_x / options.resolution) * options.resolution;
    const double origin_y = std::floor(min_y / options.resolution) * options.resolution;
    const int width = static_cast<int>(std::ceil((max_x - origin_x) / options.resolution)) + 1;
    const int height = static_cast<int>(std::ceil((max_y - origin_y) / options.resolution)) + 1;

    if (width <= 0 || height <= 0 || static_cast<long long>(width) * height > 100000000)
    {
        std::cerr << "Invalid or excessively large map dimensions: "
                  << width << " x " << height << "\n";
        return 1;
    }

    const size_t cell_count = static_cast<size_t>(width) * height;
    std::vector<unsigned char> candidate(cell_count, 0);
    for (const auto *point : points)
    {
        const int x = static_cast<int>(std::floor((point->x - origin_x) / options.resolution));
        const int y = static_cast<int>(std::floor((point->y - origin_y) / options.resolution));
        if (x >= 0 && x < width && y >= 0 && y < height)
        {
            candidate[static_cast<size_t>(y) * width + x] = 1;
        }
    }

    // Keep connected structures such as walls and tabletops, while removing
    // isolated ground returns and tiny registration artifacts.
    std::vector<unsigned char> visited(cell_count, 0);
    std::vector<unsigned char> image(cell_count, 254);
    size_t kept_components = 0;
    size_t removed_components = 0;
    for (size_t start = 0; start < cell_count; ++start)
    {
        if (!candidate[start] || visited[start]) continue;

        std::vector<size_t> component;
        std::vector<size_t> stack{start};
        visited[start] = 1;
        while (!stack.empty())
        {
            const size_t current = stack.back();
            stack.pop_back();
            component.push_back(current);

            const int x = static_cast<int>(current % width);
            const int y = static_cast<int>(current / width);
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                    const size_t neighbor = static_cast<size_t>(ny) * width + nx;
                    if (candidate[neighbor] && !visited[neighbor])
                    {
                        visited[neighbor] = 1;
                        stack.push_back(neighbor);
                    }
                }
            }
        }

        if (component.size() < static_cast<size_t>(options.min_component_cells))
        {
            ++removed_components;
            continue;
        }

        ++kept_components;
        for (const size_t cell : component)
        {
            const int x = static_cast<int>(cell % width);
            const int y = static_cast<int>(cell / width);
            // PGM rows are written top-to-bottom; Nav2's origin is bottom-left.
            image[static_cast<size_t>(height - 1 - y) * width + x] = 0;
        }
    }

    std::error_code error;
    fs::create_directories(options.output_dir, error);
    if (error)
    {
        std::cerr << "Failed to create output directory: " << error.message() << "\n";
        return 1;
    }

    const fs::path image_path = options.output_dir / "map.pgm";
    const fs::path yaml_path = options.output_dir / "map.yaml";
    std::ofstream image_file(image_path, std::ios::binary);
    if (!image_file)
    {
        std::cerr << "Failed to write " << image_path << "\n";
        return 1;
    }
    image_file << "P5\n" << width << " " << height << "\n255\n";
    image_file.write(reinterpret_cast<const char *>(image.data()),
                     static_cast<std::streamsize>(image.size()));
    image_file.close();

    std::ofstream yaml_file(yaml_path);
    if (!yaml_file)
    {
        std::cerr << "Failed to write " << yaml_path << "\n";
        return 1;
    }
    yaml_file << "image: map.pgm\n"
              << "mode: trinary\n"
              << "resolution: " << std::fixed << std::setprecision(6)
              << options.resolution << "\n"
              << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n"
              << "negate: 0\n"
              << "occupied_thresh: 0.65\n"
              << "free_thresh: 0.196\n";

    std::cout << "Wrote " << image_path << " and " << yaml_path << "\n"
              << "size: " << width << " x " << height
              << ", filtered points: " << points.size()
              << ", kept components: " << kept_components
              << ", removed components: " << removed_components << "\n"
              << "origin: [" << origin_x << ", " << origin_y << "]\n";
    return 0;
}
