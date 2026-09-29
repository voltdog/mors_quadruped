#ifndef HEIGHT_MAP_BUILDER_HEIGHT_MAP_BUILDER_HPP_
#define HEIGHT_MAP_BUILDER_HEIGHT_MAP_BUILDER_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "mors_msgs/depth_image_msg.hpp"
#include "mors_msgs/heightmap_msg.hpp"
#include "mors_msgs/pointcloud_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"

namespace hmb
{

struct ChannelsConfig
{
    std::string depth_image;
    std::string robot_state;
    std::string pointcloud;
    std::string heightmap;
};

struct CameraConfig
{
    int width{424};
    int height{240};
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double fovy_deg{58.0};
};

struct DepthConfig
{
    double min_depth{0.10};
    double max_depth{3.00};
    bool enable_downsampling{true};
    int downsample_factor{2};
    bool remove_outliers{true};
    int outlier_window_radius{1};
    double outlier_threshold_m{0.12};
    int min_neighbors_for_outlier{3};
};

struct SyncConfig
{
    double max_sync_dt_sec{0.05};
    bool require_recent_robot_state{true};
};

struct MapConfig
{
    double cell_size{0.015};
    double global_size_x{15.0};
    double global_size_y{15.0};
    bool rolling_enabled{true};
    int rolling_margin_cells_x{0};
    int rolling_margin_cells_y{0};
    int local_window_cells_x{100};
    int local_window_cells_y{100};
    double height_min{-2.0};
    double height_max{2.0};
    double height_resolution{0.005};
};

struct FilteringConfig
{
    int opening_kernel_size{5};
    std::string opening_kernel_shape{"ellipse"};
};

struct GradientConfig
{
    std::string method{"sobel"};
    double sobel_scale{1.0};
};

struct TraversabilityConfig
{
    double grad_thr_steppable{0.05};
    double grad_thr_unsteppable{0.12};
    bool unknown_is_impassable{true};
};

struct RuntimeConfig
{
    bool verbose{false};
    bool publish_pointcloud{true};
};

struct HeightMapBuilderConfig
{
    ChannelsConfig channels;
    CameraConfig camera;
    DepthConfig depth;
    SyncConfig sync;
    MapConfig map;
    FilteringConfig filtering;
    GradientConfig gradient;
    TraversabilityConfig traversability;
    RuntimeConfig runtime;
    bool use_camera_pose_params{true};
    std::array<double, 3> camera_frame_position_body_m{0.2565, 0.0, 0.0};
    std::array<double, 3> camera_frame_euler_deg{0.0, 25.0, 90.0};
    std::array<double, 3> depth_camera_position_camera_frame_m{0.0125, 0.0, 0.0};
    std::array<double, 3> depth_camera_euler_deg{-90.0, 0.0, 180.0};
    std::array<double, 16> t_body_camera{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
};

struct RobotStateSnapshot
{
    bool valid{false};
    int64_t receive_timestamp_ns{0};
    std::array<double, 3> position{0.0, 0.0, 0.0};
    std::array<double, 3> orientation_rpy{0.0, 0.0, 0.0}; // roll pitch yaw
    std::array<double, 4> orientation_quaternion{0.0, 0.0, 0.0, 1.0}; // x y z w
    double yaw{0.0};
};

class HeightMapBuilderNode
{
public:
    explicit HeightMapBuilderNode(const std::string& config_path);
    int Run();

private:
    static int64_t NowNs();
    static double ComputeYawFromQuaternion(double qx, double qy, double qz, double qw);
    static std::array<double, 9> RotationFromEuler(double roll, double pitch, double yaw);
    static std::array<double, 9> RotationFromQuaternion(double qx, double qy, double qz, double qw);
    static std::array<double, 3> Mat3MulVec3(
        const std::array<double, 9>& m,
        const std::array<double, 3>& v);
    static std::array<double, 9> Mat3MulMat3(
        const std::array<double, 9>& a,
        const std::array<double, 9>& b);
    static double DegToRad(double degrees);
    static uint16_t PackHeightCell(
        bool valid,
        uint8_t traversability_class,
        double height_m,
        double height_min,
        double height_resolution);

    bool LoadConfig(const std::string& config_path);
    void ApplyIntrinsicsFallback();
    void InitializeGlobalMapStorage();
    void ResolveRollingConfig();
    void ClearAllGlobalMapCells();
    void ResetGlobalMapAround(double center_x, double center_y);
    void ClearLogicalCell(int gx, int gy);
    void ApplyGlobalMapShift(int shift_dx_cells, int shift_dy_cells);
    void ShiftGlobalMapIfNeeded(double robot_x, double robot_y);
    int GridXFromWorldX(double x) const;
    int GridYFromWorldY(double y) const;
    size_t GridIndex(int gx, int gy) const;
    bool DecodeDepthImage(const mors_msgs::depth_image_msg& msg, std::vector<uint16_t>* depth_mm) const;
    bool IsValidDepth(double depth_m) const;
    bool IsOutlier(
        const std::vector<uint16_t>& depth_mm,
        int width,
        int height,
        int u,
        int v,
        double depth_m) const;
    void BuildPointCloudCameraFrame(
        const std::vector<uint16_t>& depth_mm,
        int width,
        int height,
        std::vector<float>* out_x,
        std::vector<float>* out_y,
        std::vector<float>* out_z) const;
    void TransformPointCloudCameraToWorld(
        const std::vector<float>& cam_x,
        const std::vector<float>& cam_y,
        const std::vector<float>& cam_z,
        std::vector<float>* world_x,
        std::vector<float>* world_y,
        std::vector<float>* world_z) const;
    void PublishPointCloud(
        const mors_msgs::depth_image_msg& depth_msg,
        const std::vector<float>& x,
        const std::vector<float>& y,
        const std::vector<float>& z);
    void UpdateGlobalHeightMap(
        const std::vector<float>& world_x,
        const std::vector<float>& world_y,
        const std::vector<float>& world_z,
        int64_t observation_timestamp_ns);
    void BuildOpeningKernelOffsets();
    void ExtractLocalHeightMapWindow(
        int start_gx,
        int start_gy,
        int local_w,
        int local_h,
        std::vector<float>* heights,
        std::vector<uint8_t>* validity) const;
    void MorphologyPass(
        const std::vector<float>& input_heights,
        const std::vector<uint8_t>& input_validity,
        int width,
        int height,
        bool is_erosion,
        std::vector<float>* output_heights,
        std::vector<uint8_t>* output_validity) const;
    void ApplyOpeningFilter(
        const std::vector<float>& input_heights,
        const std::vector<uint8_t>& input_validity,
        int width,
        int height,
        std::vector<float>* output_heights,
        std::vector<uint8_t>* output_validity) const;
    void ComputeSobelGradient(
        const std::vector<float>& input_heights,
        const std::vector<uint8_t>& input_validity,
        int width,
        int height,
        std::vector<float>* output_gradient,
        std::vector<uint8_t>* output_validity) const;
    uint8_t ClassifyTraversability(
        bool height_valid,
        double gradient_value,
        bool gradient_valid) const;
    void PublishHeightMapWindow();

    void OnDepthImage(
        const lcm::ReceiveBuffer* rbuf,
        const std::string& chan,
        const mors_msgs::depth_image_msg* msg);
    void OnRobotState(
        const lcm::ReceiveBuffer* rbuf,
        const std::string& chan,
        const mors_msgs::robot_state_msg* msg);

    HeightMapBuilderConfig config_;
    RobotStateSnapshot latest_robot_state_;

    int global_cells_x_{0};
    int global_cells_y_{0};
    double map_min_x_{0.0};
    double map_min_y_{0.0};
    int storage_offset_x_{0};
    int storage_offset_y_{0};
    int rolling_margin_cells_x_{0};
    int rolling_margin_cells_y_{0};
    std::vector<float> height_layer_;
    std::vector<uint8_t> validity_layer_;
    std::vector<int64_t> timestamp_layer_;
    std::vector<std::array<int, 2>> opening_kernel_offsets_;
    bool last_update_has_grid_bbox_{false};
    int last_update_min_gx_{0};
    int last_update_max_gx_{0};
    int last_update_min_gy_{0};
    int last_update_max_gy_{0};
    size_t last_update_count_{0};

    std::string locomotion_lcm_url_;
    lcm::LCM locomotion_lcm_;
    uint64_t frame_counter_{0};
};

}  // namespace hmb

#endif  // HEIGHT_MAP_BUILDER_HEIGHT_MAP_BUILDER_HPP_
