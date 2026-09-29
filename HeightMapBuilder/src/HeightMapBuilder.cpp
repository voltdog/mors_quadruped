#include "HeightMapBuilder/HeightMapBuilder.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include <zlib.h>

namespace
{

constexpr double kMillimetersToMeters = 1e-3;
constexpr double kSecondsToNanoseconds = 1e9;
constexpr uint8_t kClassSteppable = 0u;
constexpr uint8_t kClassUnsteppable = 1u;
constexpr uint8_t kClassImpassable = 2u;
constexpr uint16_t kHeightQMax = 0x1FFFu;
constexpr const char* kLocomotionLcmUrlEnv = "LCM_LOCOMOTION_URL";

std::string GetRequiredEnv(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0')
    {
        throw std::runtime_error(std::string("[HeightMapBuilder] ") + name + " must be set.");
    }
    return value;
}

int PositiveModulo(int value, int modulus)
{
    if (modulus <= 0)
    {
        return 0;
    }
    const int remainder = value % modulus;
    return remainder < 0 ? remainder + modulus : remainder;
}

}  // namespace

namespace hmb
{

HeightMapBuilderNode::HeightMapBuilderNode(const std::string& config_path)
    : locomotion_lcm_url_(GetRequiredEnv(kLocomotionLcmUrlEnv)),
      locomotion_lcm_(locomotion_lcm_url_)
{
    if (!locomotion_lcm_.good())
    {
        throw std::runtime_error("[HeightMapBuilder] LCM initialization failed: " + locomotion_lcm_url_);
    }

    if (!LoadConfig(config_path))
    {
        throw std::runtime_error("[HeightMapBuilder] Failed to load config: " + config_path);
    }

    locomotion_lcm_.subscribe(config_.channels.depth_image, &HeightMapBuilderNode::OnDepthImage, this);
    locomotion_lcm_.subscribe(config_.channels.robot_state, &HeightMapBuilderNode::OnRobotState, this);

    std::cout << "[HeightMapBuilder] started\n"
              << "  LCM URL: " << locomotion_lcm_url_ << "\n"
              << "  depth channel: " << config_.channels.depth_image << "\n"
              << "  robot_state channel: " << config_.channels.robot_state << "\n"
              << "  pointcloud channel: " << config_.channels.pointcloud << "\n"
              << "  heightmap channel: " << config_.channels.heightmap << "\n"
              << "  global map cells: " << global_cells_x_ << "x" << global_cells_y_ << "\n"
              << "  rolling map: " << (config_.map.rolling_enabled ? "enabled" : "disabled")
              << " margin=(" << rolling_margin_cells_x_ << "," << rolling_margin_cells_y_ << ")\n"
              << "  local window cells: " << config_.map.local_window_cells_x
              << "x" << config_.map.local_window_cells_y << "\n"
              << "  opening kernel: " << config_.filtering.opening_kernel_shape << " "
              << config_.filtering.opening_kernel_size << "x"
              << config_.filtering.opening_kernel_size << "\n"
              << "  gradient: method=" << config_.gradient.method
              << " sobel_scale=" << config_.gradient.sobel_scale << "\n"
              << "  traversability thresholds: steppable<="
              << config_.traversability.grad_thr_steppable
              << " unsteppable<=" << config_.traversability.grad_thr_unsteppable << std::endl;
}

int HeightMapBuilderNode::Run()
{
    while (true)
    {
        const int status = locomotion_lcm_.handleTimeout(10);
        if (status < 0)
        {
            std::cerr << "[HeightMapBuilder] LCM handleTimeout failed." << std::endl;
            return 1;
        }
    }
}

int64_t HeightMapBuilderNode::NowNs()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

double HeightMapBuilderNode::ComputeYawFromQuaternion(double qx, double qy, double qz, double qw)
{
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::array<double, 9> HeightMapBuilderNode::RotationFromEuler(
    double roll, double pitch, double yaw)
{
    std::array<double, 9> r{};

    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    // R_world_body (active rotation, xyz / roll-pitch-yaw)
    r[0] = cy * cp;
    r[1] = cy * sp * sr - sy * cr;
    r[2] = cy * sp * cr + sy * sr;

    r[3] = sy * cp;
    r[4] = sy * sp * sr + cy * cr;
    r[5] = sy * sp * cr - cy * sr;

    r[6] = -sp;
    r[7] = cp * sr;
    r[8] = cp * cr;

    return r;
}

std::array<double, 9> HeightMapBuilderNode::RotationFromQuaternion(
    double qx, double qy, double qz, double qw)
{
    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (!(norm > 1e-12) || !std::isfinite(norm))
    {
        return std::array<double, 9>{
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
    }

    const double x = qx / norm;
    const double y = qy / norm;
    const double z = qz / norm;
    const double w = qw / norm;

    // Matches project-wide convention in common/src/system_functions.cpp (quat2mat)
    return std::array<double, 9>{
        -1.0 + 2.0 * (w * w) + 2.0 * (x * x), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y),
        2.0 * (x * y - w * z), -1.0 + 2.0 * (w * w) + 2.0 * (y * y), 2.0 * (y * z + w * x),
        2.0 * (x * z + w * y), 2.0 * (y * z - w * x), -1.0 + 2.0 * (w * w) + 2.0 * (z * z)
    };
}

std::array<double, 3> HeightMapBuilderNode::Mat3MulVec3(
    const std::array<double, 9>& m,
    const std::array<double, 3>& v)
{
    return std::array<double, 3>{
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
    };
}

std::array<double, 9> HeightMapBuilderNode::Mat3MulMat3(
    const std::array<double, 9>& a,
    const std::array<double, 9>& b)
{
    std::array<double, 9> c{};
    for (int r = 0; r < 3; ++r)
    {
        for (int col = 0; col < 3; ++col)
        {
            c[static_cast<size_t>(r * 3 + col)] =
                a[static_cast<size_t>(r * 3 + 0)] * b[static_cast<size_t>(0 * 3 + col)] +
                a[static_cast<size_t>(r * 3 + 1)] * b[static_cast<size_t>(1 * 3 + col)] +
                a[static_cast<size_t>(r * 3 + 2)] * b[static_cast<size_t>(2 * 3 + col)];
        }
    }
    return c;
}

double HeightMapBuilderNode::DegToRad(double degrees)
{
    return degrees * std::numbers::pi_v<double> / 180.0;
}

uint16_t HeightMapBuilderNode::PackHeightCell(
    bool valid,
    uint8_t traversability_class,
    double height_m,
    double height_min,
    double height_resolution)
{
    uint16_t cell = 0u;
    if (valid)
    {
        cell |= static_cast<uint16_t>(1u << 15);
    }

    const uint16_t class_bits =
        static_cast<uint16_t>((static_cast<uint16_t>(traversability_class) & 0x3u) << 13);
    cell |= class_bits;

    if (valid && std::isfinite(height_m) && std::isfinite(height_min) &&
        std::isfinite(height_resolution) && height_resolution > 0.0)
    {
        const double q = std::round((height_m - height_min) / height_resolution);
        const int64_t q_i64 = static_cast<int64_t>(q);
        const int64_t q_clamped = std::clamp<int64_t>(q_i64, 0, static_cast<int64_t>(kHeightQMax));
        cell |= static_cast<uint16_t>(q_clamped & 0x1FFF);
    }

    return cell;
}

void HeightMapBuilderNode::InitializeGlobalMapStorage()
{
    global_cells_x_ = std::max(
        1,
        static_cast<int>(std::ceil(config_.map.global_size_x / config_.map.cell_size)));
    global_cells_y_ = std::max(
        1,
        static_cast<int>(std::ceil(config_.map.global_size_y / config_.map.cell_size)));

    map_min_x_ = -0.5 * config_.map.global_size_x;
    map_min_y_ = -0.5 * config_.map.global_size_y;
    storage_offset_x_ = 0;
    storage_offset_y_ = 0;

    const size_t total_cells =
        static_cast<size_t>(global_cells_x_) * static_cast<size_t>(global_cells_y_);
    height_layer_.assign(total_cells, std::numeric_limits<float>::quiet_NaN());
    validity_layer_.assign(total_cells, static_cast<uint8_t>(0u));
    timestamp_layer_.assign(total_cells, 0);
    last_update_has_grid_bbox_ = false;
    last_update_count_ = 0u;
}

void HeightMapBuilderNode::ResolveRollingConfig()
{
    const auto resolve_margin = [this](
                                    int configured_margin,
                                    int local_window_cells,
                                    int global_cells,
                                    const char* axis) -> int
    {
        int margin = configured_margin;
        bool auto_margin = false;
        if (margin < 0)
        {
            std::cerr << "[HeightMapBuilder] map.rolling_margin_cells_" << axis
                      << " must be >= 0. Fallback to auto." << std::endl;
            margin = 0;
        }
        if (margin == 0)
        {
            auto_margin = true;
            margin = std::max(1, local_window_cells / 2);
        }

        const int max_margin = std::max(0, (global_cells - 1) / 2);
        if (margin > max_margin)
        {
            std::cerr << "[HeightMapBuilder] map.rolling_margin_cells_" << axis
                      << (auto_margin ? " auto" : "")
                      << " value=" << margin
                      << " exceeds max allowed " << max_margin
                      << " for global_cells=" << global_cells
                      << ". Clamped." << std::endl;
            margin = max_margin;
        }
        return margin;
    };

    if (!config_.map.rolling_enabled)
    {
        rolling_margin_cells_x_ = 0;
        rolling_margin_cells_y_ = 0;
        return;
    }

    rolling_margin_cells_x_ = resolve_margin(
        config_.map.rolling_margin_cells_x,
        config_.map.local_window_cells_x,
        global_cells_x_,
        "x");
    rolling_margin_cells_y_ = resolve_margin(
        config_.map.rolling_margin_cells_y,
        config_.map.local_window_cells_y,
        global_cells_y_,
        "y");
}

void HeightMapBuilderNode::ClearAllGlobalMapCells()
{
    std::fill(height_layer_.begin(), height_layer_.end(), std::numeric_limits<float>::quiet_NaN());
    std::fill(validity_layer_.begin(), validity_layer_.end(), static_cast<uint8_t>(0u));
    std::fill(timestamp_layer_.begin(), timestamp_layer_.end(), 0);
    last_update_has_grid_bbox_ = false;
    last_update_count_ = 0u;
}

void HeightMapBuilderNode::ResetGlobalMapAround(double center_x, double center_y)
{
    map_min_x_ = center_x - 0.5 * config_.map.global_size_x;
    map_min_y_ = center_y - 0.5 * config_.map.global_size_y;
    storage_offset_x_ = 0;
    storage_offset_y_ = 0;
    ClearAllGlobalMapCells();
}

void HeightMapBuilderNode::ClearLogicalCell(int gx, int gy)
{
    if (gx < 0 || gx >= global_cells_x_ || gy < 0 || gy >= global_cells_y_)
    {
        return;
    }
    const size_t idx = GridIndex(gx, gy);
    height_layer_[idx] = std::numeric_limits<float>::quiet_NaN();
    validity_layer_[idx] = static_cast<uint8_t>(0u);
    timestamp_layer_[idx] = 0;
}

void HeightMapBuilderNode::ApplyGlobalMapShift(int shift_dx_cells, int shift_dy_cells)
{
    if (shift_dx_cells == 0 && shift_dy_cells == 0)
    {
        return;
    }

    const double old_map_min_x = map_min_x_;
    const double old_map_min_y = map_min_y_;

    map_min_x_ += static_cast<double>(shift_dx_cells) * config_.map.cell_size;
    map_min_y_ += static_cast<double>(shift_dy_cells) * config_.map.cell_size;
    storage_offset_x_ = PositiveModulo(storage_offset_x_ + shift_dx_cells, global_cells_x_);
    storage_offset_y_ = PositiveModulo(storage_offset_y_ + shift_dy_cells, global_cells_y_);

    if (shift_dx_cells > 0)
    {
        const int start_x = std::max(0, global_cells_x_ - shift_dx_cells);
        for (int gy = 0; gy < global_cells_y_; ++gy)
        {
            for (int gx = start_x; gx < global_cells_x_; ++gx)
            {
                ClearLogicalCell(gx, gy);
            }
        }
    }
    else if (shift_dx_cells < 0)
    {
        const int end_x = std::min(global_cells_x_, -shift_dx_cells);
        for (int gy = 0; gy < global_cells_y_; ++gy)
        {
            for (int gx = 0; gx < end_x; ++gx)
            {
                ClearLogicalCell(gx, gy);
            }
        }
    }

    if (shift_dy_cells > 0)
    {
        const int start_y = std::max(0, global_cells_y_ - shift_dy_cells);
        for (int gy = start_y; gy < global_cells_y_; ++gy)
        {
            for (int gx = 0; gx < global_cells_x_; ++gx)
            {
                ClearLogicalCell(gx, gy);
            }
        }
    }
    else if (shift_dy_cells < 0)
    {
        const int end_y = std::min(global_cells_y_, -shift_dy_cells);
        for (int gy = 0; gy < end_y; ++gy)
        {
            for (int gx = 0; gx < global_cells_x_; ++gx)
            {
                ClearLogicalCell(gx, gy);
            }
        }
    }

    if (config_.runtime.verbose)
    {
        std::cout << "[HeightMapBuilder] Rolling shift:"
                  << " d_cells=(" << shift_dx_cells << "," << shift_dy_cells << ")"
                  << " map_min=(" << old_map_min_x << "," << old_map_min_y << ")"
                  << " -> (" << map_min_x_ << "," << map_min_y_ << ")"
                  << " cleared_cols=" << std::abs(shift_dx_cells)
                  << " cleared_rows=" << std::abs(shift_dy_cells)
                  << std::endl;
    }
}

void HeightMapBuilderNode::ShiftGlobalMapIfNeeded(double robot_x, double robot_y)
{
    if (!config_.map.rolling_enabled)
    {
        return;
    }
    if (!std::isfinite(robot_x) || !std::isfinite(robot_y))
    {
        return;
    }
    if (global_cells_x_ <= 0 || global_cells_y_ <= 0)
    {
        return;
    }

    const int center_gx =
        static_cast<int>(std::floor((robot_x - map_min_x_) / config_.map.cell_size));
    const int center_gy =
        static_cast<int>(std::floor((robot_y - map_min_y_) / config_.map.cell_size));

    const int safe_min_x = rolling_margin_cells_x_;
    const int safe_max_x = std::max(safe_min_x, global_cells_x_ - 1 - rolling_margin_cells_x_);
    const int safe_min_y = rolling_margin_cells_y_;
    const int safe_max_y = std::max(safe_min_y, global_cells_y_ - 1 - rolling_margin_cells_y_);

    int shift_dx = 0;
    int shift_dy = 0;
    const int target_center_x = global_cells_x_ / 2;
    const int target_center_y = global_cells_y_ / 2;
    if (center_gx < safe_min_x || center_gx > safe_max_x)
    {
        shift_dx = center_gx - target_center_x;
    }
    if (center_gy < safe_min_y || center_gy > safe_max_y)
    {
        shift_dy = center_gy - target_center_y;
    }
    if (shift_dx == 0 && shift_dy == 0)
    {
        return;
    }

    if (std::abs(shift_dx) >= global_cells_x_ || std::abs(shift_dy) >= global_cells_y_)
    {
        const double old_map_min_x = map_min_x_;
        const double old_map_min_y = map_min_y_;
        ResetGlobalMapAround(robot_x, robot_y);
        if (config_.runtime.verbose)
        {
            std::cout << "[HeightMapBuilder] Rolling reset:"
                      << " requested_shift=(" << shift_dx << "," << shift_dy << ")"
                      << " map_min=(" << old_map_min_x << "," << old_map_min_y << ")"
                      << " -> (" << map_min_x_ << "," << map_min_y_ << ")"
                      << std::endl;
        }
        return;
    }

    ApplyGlobalMapShift(shift_dx, shift_dy);
}

void HeightMapBuilderNode::BuildOpeningKernelOffsets()
{
    opening_kernel_offsets_.clear();

    const int kernel_size = std::max(1, config_.filtering.opening_kernel_size);
    const int radius = kernel_size / 2;
    const bool use_rect_kernel = (config_.filtering.opening_kernel_shape == "rect");

    opening_kernel_offsets_.reserve(static_cast<size_t>(kernel_size * kernel_size));

    for (int dy = -radius; dy <= radius; ++dy)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            bool use_cell = false;
            if (use_rect_kernel || radius == 0)
            {
                use_cell = true;
            }
            else
            {
                const double nx = static_cast<double>(dx) / static_cast<double>(radius);
                const double ny = static_cast<double>(dy) / static_cast<double>(radius);
                use_cell = (nx * nx + ny * ny) <= 1.0;
            }

            if (use_cell)
            {
                opening_kernel_offsets_.push_back({dx, dy});
            }
        }
    }

    const bool has_center = std::any_of(
        opening_kernel_offsets_.begin(),
        opening_kernel_offsets_.end(),
        [](const std::array<int, 2>& offset)
        {
            return offset[0] == 0 && offset[1] == 0;
        });
    if (!has_center)
    {
        opening_kernel_offsets_.push_back({0, 0});
    }

    if (opening_kernel_offsets_.empty())
    {
        opening_kernel_offsets_.push_back({0, 0});
    }
}

int HeightMapBuilderNode::GridXFromWorldX(double x) const
{
    if (!std::isfinite(x))
    {
        return -1;
    }
    const int gx = static_cast<int>(std::floor((x - map_min_x_) / config_.map.cell_size));
    if (gx < 0 || gx >= global_cells_x_)
    {
        return -1;
    }
    return gx;
}

int HeightMapBuilderNode::GridYFromWorldY(double y) const
{
    if (!std::isfinite(y))
    {
        return -1;
    }
    const int gy = static_cast<int>(std::floor((y - map_min_y_) / config_.map.cell_size));
    if (gy < 0 || gy >= global_cells_y_)
    {
        return -1;
    }
    return gy;
}

size_t HeightMapBuilderNode::GridIndex(int gx, int gy) const
{
    if (global_cells_x_ <= 0 || global_cells_y_ <= 0)
    {
        return 0u;
    }
    const int sx = PositiveModulo(storage_offset_x_ + gx, global_cells_x_);
    const int sy = PositiveModulo(storage_offset_y_ + gy, global_cells_y_);
    return static_cast<size_t>(sy) * static_cast<size_t>(global_cells_x_) + static_cast<size_t>(sx);
}

bool HeightMapBuilderNode::LoadConfig(const std::string& config_path)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(config_path);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[HeightMapBuilder] Cannot read config '" << config_path
                  << "': " << e.what() << std::endl;
        return false;
    }

    // Имена LCM-каналов общие для всех модулей: берём их из channels.yaml рядом с конфигом.
    const std::filesystem::path channels_path =
        std::filesystem::path(config_path).parent_path() / "channels.yaml";
    YAML::Node channels;
    try
    {
        channels = YAML::LoadFile(channels_path.string());
    }
    catch (const std::exception& e)
    {
        std::cerr << "[HeightMapBuilder] Cannot read channels config '" << channels_path.string()
                  << "': " << e.what() << std::endl;
        return false;
    }
    try
    {
        config_.channels.depth_image = channels["depth_image"].as<std::string>();
        config_.channels.robot_state = channels["robot_state"].as<std::string>();
        config_.channels.pointcloud = channels["pointcloud"].as<std::string>();
        config_.channels.heightmap = channels["heightmap"].as<std::string>();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[HeightMapBuilder] Invalid channels config '" << channels_path.string()
                  << "': " << e.what() << std::endl;
        return false;
    }

    if (const YAML::Node camera = root["camera"])
    {
        if (camera["width"])
        {
            config_.camera.width = std::max(1, camera["width"].as<int>());
        }
        if (camera["height"])
        {
            config_.camera.height = std::max(1, camera["height"].as<int>());
        }
        if (camera["fx"])
        {
            config_.camera.fx = camera["fx"].as<double>();
        }
        if (camera["fy"])
        {
            config_.camera.fy = camera["fy"].as<double>();
        }
        if (camera["cx"])
        {
            config_.camera.cx = camera["cx"].as<double>();
        }
        if (camera["cy"])
        {
            config_.camera.cy = camera["cy"].as<double>();
        }
        if (camera["fovy_deg"])
        {
            config_.camera.fovy_deg = camera["fovy_deg"].as<double>();
        }
    }

    if (const YAML::Node depth = root["depth"])
    {
        if (depth["min_depth"])
        {
            config_.depth.min_depth = depth["min_depth"].as<double>();
        }
        if (depth["max_depth"])
        {
            config_.depth.max_depth = depth["max_depth"].as<double>();
        }
        if (depth["enable_downsampling"])
        {
            config_.depth.enable_downsampling = depth["enable_downsampling"].as<bool>();
        }
        if (depth["downsample_factor"])
        {
            config_.depth.downsample_factor = std::max(1, depth["downsample_factor"].as<int>());
        }
        if (depth["remove_outliers"])
        {
            config_.depth.remove_outliers = depth["remove_outliers"].as<bool>();
        }
        if (depth["outlier_window_radius"])
        {
            config_.depth.outlier_window_radius = std::max(1, depth["outlier_window_radius"].as<int>());
        }
        if (depth["outlier_threshold_m"])
        {
            config_.depth.outlier_threshold_m = std::max(0.0, depth["outlier_threshold_m"].as<double>());
        }
        if (depth["min_neighbors_for_outlier"])
        {
            config_.depth.min_neighbors_for_outlier = std::max(1, depth["min_neighbors_for_outlier"].as<int>());
        }
    }

    if (const YAML::Node sync = root["sync"])
    {
        if (sync["max_sync_dt_sec"])
        {
            config_.sync.max_sync_dt_sec = std::max(0.0, sync["max_sync_dt_sec"].as<double>());
        }
        if (sync["require_recent_robot_state"])
        {
            config_.sync.require_recent_robot_state = sync["require_recent_robot_state"].as<bool>();
        }
    }

    if (const YAML::Node map = root["map"])
    {
        if (map["cell_size"])
        {
            config_.map.cell_size = map["cell_size"].as<double>();
        }
        if (map["global_size_x"])
        {
            config_.map.global_size_x = map["global_size_x"].as<double>();
        }
        if (map["global_size_y"])
        {
            config_.map.global_size_y = map["global_size_y"].as<double>();
        }
        if (map["rolling_enabled"])
        {
            config_.map.rolling_enabled = map["rolling_enabled"].as<bool>();
        }
        if (map["rolling_margin_cells_x"])
        {
            config_.map.rolling_margin_cells_x = map["rolling_margin_cells_x"].as<int>();
        }
        if (map["rolling_margin_cells_y"])
        {
            config_.map.rolling_margin_cells_y = map["rolling_margin_cells_y"].as<int>();
        }
        if (map["local_window_cells_x"])
        {
            config_.map.local_window_cells_x = std::max(1, map["local_window_cells_x"].as<int>());
        }
        if (map["local_window_cells_y"])
        {
            config_.map.local_window_cells_y = std::max(1, map["local_window_cells_y"].as<int>());
        }
        if (map["height_min"])
        {
            config_.map.height_min = map["height_min"].as<double>();
        }
        if (map["height_max"])
        {
            config_.map.height_max = map["height_max"].as<double>();
        }
        if (map["height_resolution"])
        {
            config_.map.height_resolution = map["height_resolution"].as<double>();
        }
    }

    if (const YAML::Node filtering = root["filtering"])
    {
        if (filtering["opening_kernel_size"])
        {
            config_.filtering.opening_kernel_size = filtering["opening_kernel_size"].as<int>();
        }
        if (filtering["opening_kernel_shape"])
        {
            config_.filtering.opening_kernel_shape =
                filtering["opening_kernel_shape"].as<std::string>();
        }
    }

    if (const YAML::Node gradient = root["gradient"])
    {
        if (gradient["method"])
        {
            config_.gradient.method = gradient["method"].as<std::string>();
        }
        if (gradient["sobel_scale"])
        {
            config_.gradient.sobel_scale = gradient["sobel_scale"].as<double>();
        }
    }

    if (const YAML::Node traversability = root["traversability"])
    {
        if (traversability["grad_thr_steppable"])
        {
            config_.traversability.grad_thr_steppable =
                traversability["grad_thr_steppable"].as<double>();
        }
        if (traversability["grad_thr_unsteppable"])
        {
            config_.traversability.grad_thr_unsteppable =
                traversability["grad_thr_unsteppable"].as<double>();
        }
        if (traversability["unknown_is_impassable"])
        {
            config_.traversability.unknown_is_impassable =
                traversability["unknown_is_impassable"].as<bool>();
        }
    }

    if (const YAML::Node runtime = root["runtime"])
    {
        if (runtime["verbose"])
        {
            config_.runtime.verbose = runtime["verbose"].as<bool>();
        }
        if (runtime["publish_pointcloud"])
        {
            config_.runtime.publish_pointcloud = runtime["publish_pointcloud"].as<bool>();
        }
    }

    if (const YAML::Node transforms = root["transforms"])
    {
        if (transforms["use_camera_pose_params"])
        {
            config_.use_camera_pose_params = transforms["use_camera_pose_params"].as<bool>();
        }

        const auto parse_vec3 = [](const YAML::Node& node, std::array<double, 3>* out) -> bool
        {
            if (!node || !node.IsSequence() || node.size() != 3 || out == nullptr)
            {
                return false;
            }
            for (size_t i = 0; i < 3; ++i)
            {
                (*out)[i] = node[i].as<double>();
            }
            return true;
        };

        if (const YAML::Node camera_frame = transforms["camera_frame"])
        {
            if (camera_frame["position_body_m"] &&
                !parse_vec3(camera_frame["position_body_m"], &config_.camera_frame_position_body_m))
            {
                std::cerr << "[HeightMapBuilder] transforms.camera_frame.position_body_m must contain 3 values."
                          << std::endl;
            }
            if (camera_frame["euler_deg"] &&
                !parse_vec3(camera_frame["euler_deg"], &config_.camera_frame_euler_deg))
            {
                std::cerr << "[HeightMapBuilder] transforms.camera_frame.euler_deg must contain 3 values."
                          << std::endl;
            }
        }

        if (const YAML::Node depth_camera = transforms["depth_camera"])
        {
            if (depth_camera["position_camera_frame_m"] &&
                !parse_vec3(depth_camera["position_camera_frame_m"],
                            &config_.depth_camera_position_camera_frame_m))
            {
                std::cerr
                    << "[HeightMapBuilder] transforms.depth_camera.position_camera_frame_m must contain 3 values."
                    << std::endl;
            }
            if (depth_camera["euler_deg"] &&
                !parse_vec3(depth_camera["euler_deg"], &config_.depth_camera_euler_deg))
            {
                std::cerr << "[HeightMapBuilder] transforms.depth_camera.euler_deg must contain 3 values."
                          << std::endl;
            }
        }

        if (const YAML::Node t_body_camera = transforms["T_body_camera"])
        {
            if (!t_body_camera.IsSequence() || t_body_camera.size() != 16)
            {
                std::cerr << "[HeightMapBuilder] transforms.T_body_camera must contain 16 values. "
                             "Identity will be used."
                          << std::endl;
            }
            else
            {
                for (size_t i = 0; i < 16; ++i)
                {
                    config_.t_body_camera[i] = t_body_camera[i].as<double>();
                }
            }
        }
    }

    if (config_.use_camera_pose_params)
    {
        const std::array<double, 9> r_body_camera_frame = RotationFromEuler(
            DegToRad(config_.camera_frame_euler_deg[0]),
            DegToRad(config_.camera_frame_euler_deg[1]),
            DegToRad(config_.camera_frame_euler_deg[2]));
        const std::array<double, 9> r_camera_frame_depth = RotationFromEuler(
            DegToRad(config_.depth_camera_euler_deg[0]),
            DegToRad(config_.depth_camera_euler_deg[1]),
            DegToRad(config_.depth_camera_euler_deg[2]));

        const std::array<double, 9> r_body_camera =
            Mat3MulMat3(r_body_camera_frame, r_camera_frame_depth);
        const std::array<double, 3> rotated_offset = Mat3MulVec3(
            r_body_camera_frame, config_.depth_camera_position_camera_frame_m);
        const std::array<double, 3> t_body_camera{
            config_.camera_frame_position_body_m[0] + rotated_offset[0],
            config_.camera_frame_position_body_m[1] + rotated_offset[1],
            config_.camera_frame_position_body_m[2] + rotated_offset[2]
        };

        config_.t_body_camera = {
            r_body_camera[0], r_body_camera[1], r_body_camera[2], t_body_camera[0],
            r_body_camera[3], r_body_camera[4], r_body_camera[5], t_body_camera[1],
            r_body_camera[6], r_body_camera[7], r_body_camera[8], t_body_camera[2],
            0.0,             0.0,             0.0,             1.0
        };
    }

    if (config_.depth.min_depth >= config_.depth.max_depth)
    {
        std::cerr << "[HeightMapBuilder] Invalid depth limits: min_depth >= max_depth." << std::endl;
        return false;
    }

    if (!(config_.map.cell_size > 0.0))
    {
        std::cerr << "[HeightMapBuilder] map.cell_size must be > 0." << std::endl;
        return false;
    }
    if (!(config_.map.global_size_x > 0.0) || !(config_.map.global_size_y > 0.0))
    {
        std::cerr << "[HeightMapBuilder] map.global_size_x/y must be > 0." << std::endl;
        return false;
    }
    if (!(config_.map.height_min < config_.map.height_max))
    {
        std::cerr << "[HeightMapBuilder] map.height_min must be < map.height_max." << std::endl;
        return false;
    }
    if (!(config_.map.height_resolution > 0.0))
    {
        std::cerr << "[HeightMapBuilder] map.height_resolution must be > 0." << std::endl;
        return false;
    }
    if (config_.map.rolling_margin_cells_x < 0)
    {
        std::cerr << "[HeightMapBuilder] map.rolling_margin_cells_x must be >= 0. "
                  << "Fallback to auto." << std::endl;
        config_.map.rolling_margin_cells_x = 0;
    }
    if (config_.map.rolling_margin_cells_y < 0)
    {
        std::cerr << "[HeightMapBuilder] map.rolling_margin_cells_y must be >= 0. "
                  << "Fallback to auto." << std::endl;
        config_.map.rolling_margin_cells_y = 0;
    }
    if (config_.filtering.opening_kernel_size < 1)
    {
        std::cerr << "[HeightMapBuilder] filtering.opening_kernel_size must be >= 1. "
                  << "Fallback to 1." << std::endl;
        config_.filtering.opening_kernel_size = 1;
    }
    if ((config_.filtering.opening_kernel_size % 2) == 0)
    {
        const int normalized_size = config_.filtering.opening_kernel_size + 1;
        std::cerr << "[HeightMapBuilder] filtering.opening_kernel_size is even ("
                  << config_.filtering.opening_kernel_size
                  << "). Normalized to " << normalized_size << "." << std::endl;
        config_.filtering.opening_kernel_size = normalized_size;
    }

    std::transform(
        config_.filtering.opening_kernel_shape.begin(),
        config_.filtering.opening_kernel_shape.end(),
        config_.filtering.opening_kernel_shape.begin(),
        [](unsigned char c)
        {
            return static_cast<char>(std::tolower(c));
        });
    if (config_.filtering.opening_kernel_shape != "ellipse" &&
        config_.filtering.opening_kernel_shape != "rect")
    {
        std::cerr << "[HeightMapBuilder] Unknown filtering.opening_kernel_shape='"
                  << config_.filtering.opening_kernel_shape
                  << "'. Fallback to 'ellipse'." << std::endl;
        config_.filtering.opening_kernel_shape = "ellipse";
    }

    std::transform(
        config_.gradient.method.begin(),
        config_.gradient.method.end(),
        config_.gradient.method.begin(),
        [](unsigned char c)
        {
            return static_cast<char>(std::tolower(c));
        });
    if (config_.gradient.method != "sobel")
    {
        std::cerr << "[HeightMapBuilder] gradient.method must be 'sobel'. got='"
                  << config_.gradient.method << "'" << std::endl;
        return false;
    }
    if (!std::isfinite(config_.gradient.sobel_scale) || config_.gradient.sobel_scale <= 0.0)
    {
        std::cerr << "[HeightMapBuilder] gradient.sobel_scale must be > 0." << std::endl;
        return false;
    }
    if (!std::isfinite(config_.traversability.grad_thr_steppable) ||
        !std::isfinite(config_.traversability.grad_thr_unsteppable))
    {
        std::cerr << "[HeightMapBuilder] traversability thresholds must be finite." << std::endl;
        return false;
    }
    if (config_.traversability.grad_thr_steppable > config_.traversability.grad_thr_unsteppable)
    {
        std::cerr << "[HeightMapBuilder] traversability.grad_thr_steppable must be <= "
                  << "traversability.grad_thr_unsteppable." << std::endl;
        return false;
    }

    ApplyIntrinsicsFallback();
    InitializeGlobalMapStorage();
    ResolveRollingConfig();
    BuildOpeningKernelOffsets();
    return true;
}

void HeightMapBuilderNode::ApplyIntrinsicsFallback()
{
    if (config_.camera.fx <= 0.0 || config_.camera.fy <= 0.0)
    {
        const double fovy_rad = config_.camera.fovy_deg * std::numbers::pi_v<double> / 180.0;
        const double fy = static_cast<double>(config_.camera.height) / (2.0 * std::tan(0.5 * fovy_rad));
        config_.camera.fy = fy;
        config_.camera.fx = fy;
    }

    if (config_.camera.cx <= 0.0)
    {
        config_.camera.cx = (static_cast<double>(config_.camera.width) - 1.0) * 0.5;
    }
    if (config_.camera.cy <= 0.0)
    {
        config_.camera.cy = (static_cast<double>(config_.camera.height) - 1.0) * 0.5;
    }
}

bool HeightMapBuilderNode::DecodeDepthImage(
    const mors_msgs::depth_image_msg& msg, std::vector<uint16_t>* depth_mm) const
{
    if (depth_mm == nullptr)
    {
        return false;
    }
    if (msg.width <= 0 || msg.height <= 0)
    {
        std::cerr << "[HeightMapBuilder] Invalid depth dimensions: "
                  << msg.width << "x" << msg.height << std::endl;
        return false;
    }

    const uint64_t expected_bytes_u64 =
        static_cast<uint64_t>(msg.width) * static_cast<uint64_t>(msg.height) * sizeof(uint16_t);
    if (expected_bytes_u64 > static_cast<uint64_t>(std::numeric_limits<size_t>::max()))
    {
        std::cerr << "[HeightMapBuilder] Depth frame is too large to allocate." << std::endl;
        return false;
    }
    const size_t expected_bytes = static_cast<size_t>(expected_bytes_u64);

    if (msg.compression == 0)
    {
        if (msg.data.size() != expected_bytes)
        {
            std::cerr << "[HeightMapBuilder] Raw depth payload size mismatch. expected="
                      << expected_bytes << " got=" << msg.data.size() << std::endl;
            return false;
        }
        depth_mm->resize(static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height));
        std::memcpy(depth_mm->data(), msg.data.data(), expected_bytes);
        return true;
    }

    if (msg.compression == 1)
    {
        std::vector<uint8_t> raw_bytes(expected_bytes);
        uLongf decoded_size = static_cast<uLongf>(raw_bytes.size());
        const int zret = ::uncompress(
            raw_bytes.data(),
            &decoded_size,
            reinterpret_cast<const Bytef*>(msg.data.data()),
            static_cast<uLongf>(msg.data.size()));
        if (zret != Z_OK)
        {
            std::cerr << "[HeightMapBuilder] zlib uncompress failed. code=" << zret << std::endl;
            return false;
        }
        if (static_cast<size_t>(decoded_size) != expected_bytes)
        {
            std::cerr << "[HeightMapBuilder] Decompressed depth payload size mismatch. expected="
                      << expected_bytes << " got=" << decoded_size << std::endl;
            return false;
        }
        depth_mm->resize(static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height));
        std::memcpy(depth_mm->data(), raw_bytes.data(), expected_bytes);
        return true;
    }

    std::cerr << "[HeightMapBuilder] Unsupported depth compression type: "
              << static_cast<int>(msg.compression) << std::endl;
    return false;
}

bool HeightMapBuilderNode::IsValidDepth(double depth_m) const
{
    return std::isfinite(depth_m) &&
           depth_m >= config_.depth.min_depth &&
           depth_m <= config_.depth.max_depth;
}

bool HeightMapBuilderNode::IsOutlier(
    const std::vector<uint16_t>& depth_mm,
    int width,
    int height,
    int u,
    int v,
    double depth_m) const
{
    if (!config_.depth.remove_outliers)
    {
        return false;
    }

    const int r = config_.depth.outlier_window_radius;
    std::vector<double> neighbors;
    neighbors.reserve(static_cast<size_t>((2 * r + 1) * (2 * r + 1)));

    for (int dv = -r; dv <= r; ++dv)
    {
        const int vv = v + dv;
        if (vv < 0 || vv >= height)
        {
            continue;
        }
        for (int du = -r; du <= r; ++du)
        {
            const int uu = u + du;
            if (uu < 0 || uu >= width || (du == 0 && dv == 0))
            {
                continue;
            }
            const uint16_t raw_depth = depth_mm[static_cast<size_t>(vv) * static_cast<size_t>(width) + static_cast<size_t>(uu)];
            if (raw_depth == 0u)
            {
                continue;
            }

            const double neighbor_depth_m = static_cast<double>(raw_depth) * kMillimetersToMeters;
            if (!IsValidDepth(neighbor_depth_m))
            {
                continue;
            }
            neighbors.push_back(neighbor_depth_m);
        }
    }

    if (static_cast<int>(neighbors.size()) < config_.depth.min_neighbors_for_outlier)
    {
        return false;
    }

    auto middle_it = neighbors.begin() + (neighbors.size() / 2u);
    std::nth_element(neighbors.begin(), middle_it, neighbors.end());
    const double median = *middle_it;
    return std::fabs(depth_m - median) > config_.depth.outlier_threshold_m;
}

void HeightMapBuilderNode::BuildPointCloudCameraFrame(
    const std::vector<uint16_t>& depth_mm,
    int width,
    int height,
    std::vector<float>* out_x,
    std::vector<float>* out_y,
    std::vector<float>* out_z) const
{
    if (out_x == nullptr || out_y == nullptr || out_z == nullptr)
    {
        return;
    }

    out_x->clear();
    out_y->clear();
    out_z->clear();

    const int downsample =
        config_.depth.enable_downsampling ? std::max(1, config_.depth.downsample_factor) : 1;

    const size_t approx_points =
        (static_cast<size_t>(width) / static_cast<size_t>(downsample) + 1u) *
        (static_cast<size_t>(height) / static_cast<size_t>(downsample) + 1u);
    out_x->reserve(approx_points);
    out_y->reserve(approx_points);
    out_z->reserve(approx_points);

    for (int v = 0; v < height; v += downsample)
    {
        for (int u = 0; u < width; u += downsample)
        {
            const uint16_t raw_depth = depth_mm[static_cast<size_t>(v) * static_cast<size_t>(width) + static_cast<size_t>(u)];
            if (raw_depth == 0u)
            {
                continue;
            }

            const double depth_m = static_cast<double>(raw_depth) * kMillimetersToMeters;
            if (!IsValidDepth(depth_m))
            {
                continue;
            }
            if (IsOutlier(depth_mm, width, height, u, v, depth_m))
            {
                continue;
            }

            const double x = (static_cast<double>(u) - config_.camera.cx) * depth_m / config_.camera.fx;
            const double y = (static_cast<double>(v) - config_.camera.cy) * depth_m / config_.camera.fy;

            out_x->push_back(static_cast<float>(x));
            out_y->push_back(static_cast<float>(y));
            out_z->push_back(static_cast<float>(depth_m));
        }
    }
}

void HeightMapBuilderNode::TransformPointCloudCameraToWorld(
    const std::vector<float>& cam_x,
    const std::vector<float>& cam_y,
    const std::vector<float>& cam_z,
    std::vector<float>* world_x,
    std::vector<float>* world_y,
    std::vector<float>* world_z) const
{
    if (world_x == nullptr || world_y == nullptr || world_z == nullptr)
    {
        return;
    }
    if (cam_x.size() != cam_y.size() || cam_x.size() != cam_z.size())
    {
        world_x->clear();
        world_y->clear();
        world_z->clear();
        return;
    }

    const std::array<double, 9> r_body_camera{
        config_.t_body_camera[0], config_.t_body_camera[1], config_.t_body_camera[2],
        config_.t_body_camera[4], config_.t_body_camera[5], config_.t_body_camera[6],
        config_.t_body_camera[8], config_.t_body_camera[9], config_.t_body_camera[10]
    };
    const std::array<double, 3> t_body_camera{
        config_.t_body_camera[3], config_.t_body_camera[7], config_.t_body_camera[11]
    };

    std::array<double, 9> r_world_body{};
    const bool euler_valid =
        std::isfinite(latest_robot_state_.orientation_rpy[0]) &&
        std::isfinite(latest_robot_state_.orientation_rpy[1]) &&
        std::isfinite(latest_robot_state_.orientation_rpy[2]);
    if (euler_valid)
    {
        r_world_body = RotationFromEuler(
            latest_robot_state_.orientation_rpy[0],
            latest_robot_state_.orientation_rpy[1],
            latest_robot_state_.orientation_rpy[2]);
    }
    else
    {
        r_world_body = RotationFromQuaternion(
            latest_robot_state_.orientation_quaternion[0],
            latest_robot_state_.orientation_quaternion[1],
            latest_robot_state_.orientation_quaternion[2],
            latest_robot_state_.orientation_quaternion[3]);
    }

    const std::array<double, 3> t_world_body{
        latest_robot_state_.position[0],
        latest_robot_state_.position[1],
        latest_robot_state_.position[2]
    };

    world_x->resize(cam_x.size());
    world_y->resize(cam_x.size());
    world_z->resize(cam_x.size());

    for (size_t i = 0; i < cam_x.size(); ++i)
    {
        const std::array<double, 3> p_camera{
            static_cast<double>(cam_x[i]),
            static_cast<double>(cam_y[i]),
            static_cast<double>(cam_z[i])
        };
        std::array<double, 3> p_body = Mat3MulVec3(r_body_camera, p_camera);
        p_body[0] += t_body_camera[0];
        p_body[1] += t_body_camera[1];
        p_body[2] += t_body_camera[2];

        std::array<double, 3> p_world = Mat3MulVec3(r_world_body, p_body);
        p_world[0] += t_world_body[0];
        p_world[1] += t_world_body[1];
        p_world[2] += t_world_body[2];

        (*world_x)[i] = static_cast<float>(p_world[0]);
        (*world_y)[i] = static_cast<float>(p_world[1]);
        (*world_z)[i] = static_cast<float>(p_world[2]);
    }
}

void HeightMapBuilderNode::PublishPointCloud(
    const mors_msgs::depth_image_msg& depth_msg,
    const std::vector<float>& x,
    const std::vector<float>& y,
    const std::vector<float>& z)
{
    if (x.size() != y.size() || x.size() != z.size())
    {
        std::cerr << "[HeightMapBuilder] Internal pointcloud vector size mismatch." << std::endl;
        return;
    }

    mors_msgs::pointcloud_msg pointcloud_msg;
    pointcloud_msg.timestamp = NowNs();
    pointcloud_msg.depth_timestamp = depth_msg.timestamp;
    pointcloud_msg.robot_state_rx_timestamp = latest_robot_state_.receive_timestamp_ns;

    for (size_t i = 0; i < 3; ++i)
    {
        pointcloud_msg.robot_position[i] = static_cast<float>(latest_robot_state_.position[i]);
    }
    for (size_t i = 0; i < 4; ++i)
    {
        pointcloud_msg.robot_orientation_quaternion[i] =
            static_cast<float>(latest_robot_state_.orientation_quaternion[i]);
    }
    pointcloud_msg.robot_yaw = static_cast<float>(latest_robot_state_.yaw);

    pointcloud_msg.points_count = static_cast<int32_t>(x.size());
    pointcloud_msg.x = x;
    pointcloud_msg.y = y;
    pointcloud_msg.z = z;

    locomotion_lcm_.publish(config_.channels.pointcloud, &pointcloud_msg);
}

void HeightMapBuilderNode::UpdateGlobalHeightMap(
    const std::vector<float>& world_x,
    const std::vector<float>& world_y,
    const std::vector<float>& world_z,
    int64_t observation_timestamp_ns)
{
    if (world_x.size() != world_y.size() || world_x.size() != world_z.size())
    {
        std::cerr << "[HeightMapBuilder] Cannot update map: world point vectors size mismatch."
                  << std::endl;
        return;
    }

    size_t skipped_height_count = 0u;
    size_t skipped_xy_count = 0u;
    size_t updated_count = 0u;
    bool has_world_bounds = false;
    double min_world_x = 0.0;
    double max_world_x = 0.0;
    double min_world_y = 0.0;
    double max_world_y = 0.0;
    double min_world_z = 0.0;
    double max_world_z = 0.0;
    int min_gx = 0;
    int max_gx = 0;
    int min_gy = 0;
    int max_gy = 0;
    bool has_grid_bounds = false;

    for (size_t i = 0; i < world_x.size(); ++i)
    {
        const double x = static_cast<double>(world_x[i]);
        const double y = static_cast<double>(world_y[i]);
        const double z = static_cast<double>(world_z[i]);

        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
        {
            if (!has_world_bounds)
            {
                min_world_x = x;
                max_world_x = x;
                min_world_y = y;
                max_world_y = y;
                min_world_z = z;
                max_world_z = z;
                has_world_bounds = true;
            }
            else
            {
                min_world_x = std::min(min_world_x, x);
                max_world_x = std::max(max_world_x, x);
                min_world_y = std::min(min_world_y, y);
                max_world_y = std::max(max_world_y, y);
                min_world_z = std::min(min_world_z, z);
                max_world_z = std::max(max_world_z, z);
            }
        }

        if (!std::isfinite(z) || z < config_.map.height_min || z > config_.map.height_max)
        {
            ++skipped_height_count;
            continue;
        }

        const int gx = GridXFromWorldX(x);
        const int gy = GridYFromWorldY(y);
        if (gx < 0 || gy < 0)
        {
            ++skipped_xy_count;
            continue;
        }

        const size_t idx = GridIndex(gx, gy);
        height_layer_[idx] = static_cast<float>(z);
        validity_layer_[idx] = static_cast<uint8_t>(1u);
        timestamp_layer_[idx] = observation_timestamp_ns;
        ++updated_count;

        if (!has_grid_bounds)
        {
            min_gx = gx;
            max_gx = gx;
            min_gy = gy;
            max_gy = gy;
            has_grid_bounds = true;
        }
        else
        {
            min_gx = std::min(min_gx, gx);
            max_gx = std::max(max_gx, gx);
            min_gy = std::min(min_gy, gy);
            max_gy = std::max(max_gy, gy);
        }
    }

    last_update_has_grid_bbox_ = has_grid_bounds;
    if (has_grid_bounds)
    {
        last_update_min_gx_ = min_gx;
        last_update_max_gx_ = max_gx;
        last_update_min_gy_ = min_gy;
        last_update_max_gy_ = max_gy;
    }
    last_update_count_ = updated_count;

    if (config_.runtime.verbose && frame_counter_ % 30u == 0u)
    {
        std::cout << "[HeightMapBuilder] Map update stats:"
                  << " total=" << world_x.size()
                  << " updated=" << updated_count
                  << " skipped_height=" << skipped_height_count
                  << " skipped_xy=" << skipped_xy_count;
        if (has_world_bounds)
        {
            std::cout << " world_x=[" << min_world_x << "," << max_world_x << "]"
                      << " world_y=[" << min_world_y << "," << max_world_y << "]"
                      << " world_z=[" << min_world_z << "," << max_world_z << "]";
        }
        if (has_grid_bounds)
        {
            std::cout << " grid_bbox=[(" << min_gx << "," << min_gy
                      << ")-(" << max_gx << "," << max_gy << ")]";
        }
        std::cout << std::endl;
    }
}

void HeightMapBuilderNode::ExtractLocalHeightMapWindow(
    int start_gx,
    int start_gy,
    int local_w,
    int local_h,
    std::vector<float>* heights,
    std::vector<uint8_t>* validity) const
{
    if (heights == nullptr || validity == nullptr)
    {
        return;
    }

    const int64_t data_size_i64 = static_cast<int64_t>(local_w) * static_cast<int64_t>(local_h);
    if (local_w <= 0 || local_h <= 0 || data_size_i64 <= 0)
    {
        heights->clear();
        validity->clear();
        return;
    }
    const uint64_t data_size_u64 = static_cast<uint64_t>(data_size_i64);
    if (data_size_u64 > static_cast<uint64_t>(std::numeric_limits<size_t>::max()))
    {
        heights->clear();
        validity->clear();
        return;
    }
    const size_t data_size = static_cast<size_t>(data_size_u64);
    heights->assign(data_size, std::numeric_limits<float>::quiet_NaN());
    validity->assign(data_size, static_cast<uint8_t>(0u));

    for (int ly = 0; ly < local_h; ++ly)
    {
        for (int lx = 0; lx < local_w; ++lx)
        {
            const size_t out_idx =
                static_cast<size_t>(ly) * static_cast<size_t>(local_w) + static_cast<size_t>(lx);
            const int gx = start_gx + lx;
            const int gy = start_gy + ly;

            if (gx < 0 || gx >= global_cells_x_ || gy < 0 || gy >= global_cells_y_)
            {
                continue;
            }

            const size_t map_idx = GridIndex(gx, gy);
            const bool valid =
                validity_layer_[map_idx] != 0u && std::isfinite(height_layer_[map_idx]);
            if (!valid)
            {
                continue;
            }

            (*validity)[out_idx] = static_cast<uint8_t>(1u);
            (*heights)[out_idx] = height_layer_[map_idx];
        }
    }
}

void HeightMapBuilderNode::MorphologyPass(
    const std::vector<float>& input_heights,
    const std::vector<uint8_t>& input_validity,
    int width,
    int height,
    bool is_erosion,
    std::vector<float>* output_heights,
    std::vector<uint8_t>* output_validity) const
{
    if (output_heights == nullptr || output_validity == nullptr)
    {
        return;
    }

    const int64_t data_size_i64 = static_cast<int64_t>(width) * static_cast<int64_t>(height);
    if (width <= 0 || height <= 0 || data_size_i64 <= 0)
    {
        output_heights->clear();
        output_validity->clear();
        return;
    }
    const uint64_t data_size_u64 = static_cast<uint64_t>(data_size_i64);
    if (data_size_u64 > static_cast<uint64_t>(std::numeric_limits<size_t>::max()))
    {
        output_heights->clear();
        output_validity->clear();
        return;
    }
    const size_t data_size = static_cast<size_t>(data_size_u64);
    if (input_heights.size() != data_size || input_validity.size() != data_size)
    {
        output_heights->clear();
        output_validity->clear();
        return;
    }

    output_heights->assign(data_size, std::numeric_limits<float>::quiet_NaN());
    output_validity->assign(data_size, static_cast<uint8_t>(0u));

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const size_t center_idx =
                static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);

            // Unknown cells should remain unknown after each morphology pass.
            if (input_validity[center_idx] == 0u || !std::isfinite(input_heights[center_idx]))
            {
                continue;
            }

            float aggregate = input_heights[center_idx];

            for (const auto& offset : opening_kernel_offsets_)
            {
                const int nx = x + offset[0];
                const int ny = y + offset[1];
                if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                {
                    continue;
                }

                const size_t neighbor_idx =
                    static_cast<size_t>(ny) * static_cast<size_t>(width) + static_cast<size_t>(nx);
                if (input_validity[neighbor_idx] == 0u)
                {
                    continue;
                }

                const float neighbor_h = input_heights[neighbor_idx];
                if (!std::isfinite(neighbor_h))
                {
                    continue;
                }

                aggregate = is_erosion
                                ? std::min(aggregate, neighbor_h)
                                : std::max(aggregate, neighbor_h);
            }

            (*output_validity)[center_idx] = static_cast<uint8_t>(1u);
            (*output_heights)[center_idx] = aggregate;
        }
    }
}

void HeightMapBuilderNode::ApplyOpeningFilter(
    const std::vector<float>& input_heights,
    const std::vector<uint8_t>& input_validity,
    int width,
    int height,
    std::vector<float>* output_heights,
    std::vector<uint8_t>* output_validity) const
{
    if (output_heights == nullptr || output_validity == nullptr)
    {
        return;
    }

    if (config_.filtering.opening_kernel_size <= 1 || opening_kernel_offsets_.empty())
    {
        *output_heights = input_heights;
        *output_validity = input_validity;
        return;
    }

    std::vector<float> eroded_heights;
    std::vector<uint8_t> eroded_validity;
    MorphologyPass(
        input_heights,
        input_validity,
        width,
        height,
        true,
        &eroded_heights,
        &eroded_validity);

    MorphologyPass(
        eroded_heights,
        eroded_validity,
        width,
        height,
        false,
        output_heights,
        output_validity);
}

void HeightMapBuilderNode::ComputeSobelGradient(
    const std::vector<float>& input_heights,
    const std::vector<uint8_t>& input_validity,
    int width,
    int height,
    std::vector<float>* output_gradient,
    std::vector<uint8_t>* output_validity) const
{
    if (output_gradient == nullptr || output_validity == nullptr)
    {
        return;
    }

    const int64_t data_size_i64 = static_cast<int64_t>(width) * static_cast<int64_t>(height);
    if (width <= 0 || height <= 0 || data_size_i64 <= 0)
    {
        output_gradient->clear();
        output_validity->clear();
        return;
    }
    const uint64_t data_size_u64 = static_cast<uint64_t>(data_size_i64);
    if (data_size_u64 > static_cast<uint64_t>(std::numeric_limits<size_t>::max()))
    {
        output_gradient->clear();
        output_validity->clear();
        return;
    }
    const size_t data_size = static_cast<size_t>(data_size_u64);
    if (input_heights.size() != data_size || input_validity.size() != data_size)
    {
        output_gradient->clear();
        output_validity->clear();
        return;
    }

    output_gradient->assign(data_size, std::numeric_limits<float>::quiet_NaN());
    output_validity->assign(data_size, static_cast<uint8_t>(0u));

    const double denom = 8.0 * config_.map.cell_size;
    if (!(denom > 0.0) || !std::isfinite(denom))
    {
        return;
    }

    for (int y = 1; y < height - 1; ++y)
    {
        for (int x = 1; x < width - 1; ++x)
        {
            const size_t center_idx =
                static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
            if (input_validity[center_idx] == 0u || !std::isfinite(input_heights[center_idx]))
            {
                continue;
            }

            double h[3][3]{};
            bool neighborhood_valid = true;
            for (int ky = -1; ky <= 1 && neighborhood_valid; ++ky)
            {
                for (int kx = -1; kx <= 1; ++kx)
                {
                    const int nx = x + kx;
                    const int ny = y + ky;
                    const size_t idx =
                        static_cast<size_t>(ny) * static_cast<size_t>(width) + static_cast<size_t>(nx);
                    if (input_validity[idx] == 0u || !std::isfinite(input_heights[idx]))
                    {
                        neighborhood_valid = false;
                        break;
                    }
                    h[ky + 1][kx + 1] = static_cast<double>(input_heights[idx]);
                }
            }

            if (!neighborhood_valid)
            {
                continue;
            }

            const double gx_num =
                -h[0][0] + h[0][2] - 2.0 * h[1][0] + 2.0 * h[1][2] - h[2][0] + h[2][2];
            const double gy_num =
                -h[0][0] - 2.0 * h[0][1] - h[0][2] + h[2][0] + 2.0 * h[2][1] + h[2][2];
            const double gx = gx_num / denom;
            const double gy = gy_num / denom;
            const double grad = config_.gradient.sobel_scale * std::hypot(gx, gy);
            if (!std::isfinite(grad))
            {
                continue;
            }

            (*output_validity)[center_idx] = static_cast<uint8_t>(1u);
            (*output_gradient)[center_idx] = static_cast<float>(grad);
        }
    }
}

uint8_t HeightMapBuilderNode::ClassifyTraversability(
    bool height_valid,
    double gradient_value,
    bool gradient_valid) const
{
    if (!height_valid)
    {
        return config_.traversability.unknown_is_impassable ? kClassImpassable : kClassUnsteppable;
    }
    if (!gradient_valid || !std::isfinite(gradient_value))
    {
        // Border of the local window may not have a full Sobel neighborhood.
        // Keep it non-impassable to avoid false hard obstacles.
        return kClassUnsteppable;
    }

    if (gradient_value <= config_.traversability.grad_thr_steppable)
    {
        return kClassSteppable;
    }
    if (gradient_value <= config_.traversability.grad_thr_unsteppable)
    {
        return kClassUnsteppable;
    }
    return kClassImpassable;
}

void HeightMapBuilderNode::PublishHeightMapWindow()
{
    const int local_w = std::max(1, config_.map.local_window_cells_x);
    const int local_h = std::max(1, config_.map.local_window_cells_y);
    const int64_t data_size_i64 = static_cast<int64_t>(local_w) * static_cast<int64_t>(local_h);
    if (data_size_i64 <= 0 || data_size_i64 > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
    {
        std::cerr << "[HeightMapBuilder] Local window size is invalid: "
                  << local_w << "x" << local_h << std::endl;
        return;
    }
    const size_t data_size = static_cast<size_t>(data_size_i64);

    const double robot_x = latest_robot_state_.position[0];
    const double robot_y = latest_robot_state_.position[1];
    const double safe_robot_x = std::isfinite(robot_x) ? robot_x : 0.0;
    const double safe_robot_y = std::isfinite(robot_y) ? robot_y : 0.0;
    const int center_gx =
        static_cast<int>(std::floor((safe_robot_x - map_min_x_) / config_.map.cell_size));
    const int center_gy =
        static_cast<int>(std::floor((safe_robot_y - map_min_y_) / config_.map.cell_size));
    const int clamped_center_gx = std::clamp(center_gx, 0, std::max(0, global_cells_x_ - 1));
    const int clamped_center_gy = std::clamp(center_gy, 0, std::max(0, global_cells_y_ - 1));

    int start_gx = clamped_center_gx - (local_w / 2);
    int start_gy = clamped_center_gy - (local_h / 2);
    if (global_cells_x_ > local_w)
    {
        start_gx = std::clamp(start_gx, 0, global_cells_x_ - local_w);
    }
    else
    {
        start_gx = 0;
    }
    if (global_cells_y_ > local_h)
    {
        start_gy = std::clamp(start_gy, 0, global_cells_y_ - local_h);
    }
    else
    {
        start_gy = 0;
    }

    std::vector<float> window_heights(data_size, std::numeric_limits<float>::quiet_NaN());
    std::vector<uint8_t> window_validity(data_size, static_cast<uint8_t>(0u));
    for (int ly = 0; ly < local_h; ++ly)
    {
        for (int lx = 0; lx < local_w; ++lx)
        {
            const int gx = start_gx + lx;
            const int gy = start_gy + ly;
            if (gx < 0 || gx >= global_cells_x_ || gy < 0 || gy >= global_cells_y_)
            {
                continue;
            }

            const size_t map_idx = GridIndex(gx, gy);
            if (validity_layer_[map_idx] == 0u || !std::isfinite(height_layer_[map_idx]))
            {
                continue;
            }

            const size_t out_idx =
                static_cast<size_t>(ly) * static_cast<size_t>(local_w) + static_cast<size_t>(lx);
            window_validity[out_idx] = static_cast<uint8_t>(1u);
            window_heights[out_idx] = height_layer_[map_idx];
        }
    }

    std::vector<float> filtered_heights;
    std::vector<uint8_t> filtered_validity;
    ApplyOpeningFilter(
        window_heights,
        window_validity,
        local_w,
        local_h,
        &filtered_heights,
        &filtered_validity);

    const size_t raw_valid_count = static_cast<size_t>(std::count_if(
        window_validity.begin(),
        window_validity.end(),
        [](uint8_t v)
        {
            return v != 0u;
        }));
    const size_t filtered_valid_count = static_cast<size_t>(std::count_if(
        filtered_validity.begin(),
        filtered_validity.end(),
        [](uint8_t v)
        {
            return v != 0u;
        }));
    if (filtered_valid_count == 0u && raw_valid_count > 0u)
    {
        if (config_.runtime.verbose && frame_counter_ % 30u == 0u)
        {
            std::cerr << "[HeightMapBuilder] opening filter removed all valid cells. "
                      << "Fallback to raw local window for publish." << std::endl;
        }
        filtered_heights = window_heights;
        filtered_validity = window_validity;
    }

    if (config_.runtime.verbose && frame_counter_ % 30u == 0u)
    {
        size_t direct_window_valid_count = 0u;
        const int end_gx = start_gx + local_w;
        const int end_gy = start_gy + local_h;
        for (int gy = start_gy; gy < end_gy; ++gy)
        {
            for (int gx = start_gx; gx < end_gx; ++gx)
            {
                if (gx < 0 || gx >= global_cells_x_ || gy < 0 || gy >= global_cells_y_)
                {
                    continue;
                }
                const size_t idx = GridIndex(gx, gy);
                if (validity_layer_[idx] != 0u && std::isfinite(height_layer_[idx]))
                {
                    ++direct_window_valid_count;
                }
            }
        }

        int overlap_min_gx = 0;
        int overlap_max_gx = -1;
        int overlap_min_gy = 0;
        int overlap_max_gy = -1;
        if (last_update_has_grid_bbox_)
        {
            overlap_min_gx = std::max(start_gx, last_update_min_gx_);
            overlap_max_gx = std::min(end_gx - 1, last_update_max_gx_);
            overlap_min_gy = std::max(start_gy, last_update_min_gy_);
            overlap_max_gy = std::min(end_gy - 1, last_update_max_gy_);
        }

        std::cout << "[HeightMapBuilder] HEIGHTMAP window stats:"
                  << " raw_valid=" << raw_valid_count
                  << " filtered_valid=" << filtered_valid_count
                  << " direct_window_valid=" << direct_window_valid_count
                  << " last_update_count=" << last_update_count_
                  << " center_g=(" << center_gx << "," << center_gy << ")"
                  << " start_g=(" << start_gx << "," << start_gy << ")"
                  << " overlap=[(" << overlap_min_gx << "," << overlap_min_gy
                  << ")-(" << overlap_max_gx << "," << overlap_max_gy << ")]"
                  << std::endl;
    }

    std::vector<float> gradient_values;
    std::vector<uint8_t> gradient_validity;
    ComputeSobelGradient(
        filtered_heights,
        filtered_validity,
        local_w,
        local_h,
        &gradient_values,
        &gradient_validity);

    mors_msgs::heightmap_msg msg;
    msg.origin_x = static_cast<float>(safe_robot_x);
    msg.origin_y = static_cast<float>(safe_robot_y);
    msg.yaw = static_cast<float>(latest_robot_state_.yaw);
    msg.data_size = static_cast<int32_t>(data_size);
    msg.data.resize(data_size);

    for (int ly = 0; ly < local_h; ++ly)
    {
        for (int lx = 0; lx < local_w; ++lx)
        {
            const size_t out_idx =
                static_cast<size_t>(ly) * static_cast<size_t>(local_w) + static_cast<size_t>(lx);
            const bool valid =
                out_idx < filtered_validity.size() &&
                filtered_validity[out_idx] != 0u &&
                out_idx < filtered_heights.size() &&
                std::isfinite(filtered_heights[out_idx]);
            const double h = valid ? static_cast<double>(filtered_heights[out_idx]) : 0.0;
            const bool gradient_valid =
                valid &&
                out_idx < gradient_validity.size() &&
                gradient_validity[out_idx] != 0u &&
                out_idx < gradient_values.size() &&
                std::isfinite(gradient_values[out_idx]);
            const double gradient_value = gradient_valid
                                              ? static_cast<double>(gradient_values[out_idx])
                                              : std::numeric_limits<double>::quiet_NaN();
            const uint8_t cls = ClassifyTraversability(valid, gradient_value, gradient_valid);

            msg.data[out_idx] = static_cast<int16_t>(PackHeightCell(
                valid,
                cls,
                h,
                config_.map.height_min,
                config_.map.height_resolution));
        }
    }

    locomotion_lcm_.publish(config_.channels.heightmap, &msg);
}

void HeightMapBuilderNode::OnDepthImage(
    const lcm::ReceiveBuffer* /*rbuf*/,
    const std::string& /*chan*/,
    const mors_msgs::depth_image_msg* msg)
{
    if (msg == nullptr)
    {
        return;
    }

    if (config_.sync.require_recent_robot_state && !latest_robot_state_.valid)
    {
        return;
    }

    const int64_t depth_receive_ts_ns = NowNs();
    if (latest_robot_state_.valid)
    {
        const int64_t dt_ns = std::llabs(depth_receive_ts_ns - latest_robot_state_.receive_timestamp_ns);
        const double dt_sec = static_cast<double>(dt_ns) / kSecondsToNanoseconds;
        if (dt_sec > config_.sync.max_sync_dt_sec && config_.sync.require_recent_robot_state)
        {
            return;
        }
    }

    std::vector<uint16_t> depth_mm;
    if (!DecodeDepthImage(*msg, &depth_mm))
    {
        return;
    }

    std::vector<float> points_x;
    std::vector<float> points_y;
    std::vector<float> points_z;
    BuildPointCloudCameraFrame(depth_mm, msg->width, msg->height, &points_x, &points_y, &points_z);

    std::vector<float> points_world_x;
    std::vector<float> points_world_y;
    std::vector<float> points_world_z;
    TransformPointCloudCameraToWorld(
        points_x,
        points_y,
        points_z,
        &points_world_x,
        &points_world_y,
        &points_world_z);
    if (config_.runtime.publish_pointcloud)
    {
        PublishPointCloud(*msg, points_world_x, points_world_y, points_world_z);
    }
    ShiftGlobalMapIfNeeded(
        latest_robot_state_.position[0],
        latest_robot_state_.position[1]);
    UpdateGlobalHeightMap(
        points_world_x,
        points_world_y,
        points_world_z,
        depth_receive_ts_ns);
    PublishHeightMapWindow();

    ++frame_counter_;
    if (config_.runtime.verbose && frame_counter_ % 30u == 0u)
    {
        std::cout << "[HeightMapBuilder] Frame " << frame_counter_
                  << ", points: " << points_x.size() << std::endl;
    }
}

void HeightMapBuilderNode::OnRobotState(
    const lcm::ReceiveBuffer* /*rbuf*/,
    const std::string& /*chan*/,
    const mors_msgs::robot_state_msg* msg)
{
    if (msg == nullptr)
    {
        return;
    }

    latest_robot_state_.receive_timestamp_ns = NowNs();

    for (size_t i = 0; i < 3; ++i)
    {
        latest_robot_state_.position[i] = msg->body.position[i];
        latest_robot_state_.orientation_rpy[i] = msg->body.orientation[i];
    }
    for (size_t i = 0; i < 4; ++i)
    {
        latest_robot_state_.orientation_quaternion[i] = msg->body.orientation_quaternion[i];
    }

    const double yaw_from_euler = msg->body.orientation[2];
    if (std::isfinite(yaw_from_euler))
    {
        latest_robot_state_.yaw = yaw_from_euler;
    }
    else
    {
        latest_robot_state_.yaw = ComputeYawFromQuaternion(
            latest_robot_state_.orientation_quaternion[0],
            latest_robot_state_.orientation_quaternion[1],
            latest_robot_state_.orientation_quaternion[2],
            latest_robot_state_.orientation_quaternion[3]);
    }

    latest_robot_state_.valid = true;
}

}  // namespace hmb
