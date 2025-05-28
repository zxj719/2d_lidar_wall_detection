#ifndef WALL_DETECTOR_H
#define WALL_DETECTOR_H

#include <Eigen/Dense>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef std::vector<Eigen::Vector4f> TimedPointCloud;

class WallDetector {
private:
    std::vector<std::array<float, 3>> m_wall_lines;
    std::array<float, 3> m_wall_closest_line;
    TimedPointCloud m_wall_points;
    TimedPointCloud m_noise_points;
    
    // Helper functions
    float PointDistance(const Eigen::Vector4f& p1, const Eigen::Vector4f& p2);
    std::pair<Eigen::Vector2f, float> LeastSquaresFit(const Eigen::MatrixXf& points);
    float PointFitVar(const Eigen::Vector2f& point, const Eigen::Vector2f& coeffs);

public:
    WallDetector();
    ~WallDetector() = default;
    
    // Main detection function with debug output
    void DetectWalls(
        const TimedPointCloud& points,
        float max_fit_var = 0.0001f,
        float min_span_distance = 0.3f,
        float min_distance = 0.015f,
        float max_distance = 0.2f,
        int segment_window = 20
    );
    
    // Function to find closest wall line
    void FindClosestWall(float robot_radius);
    
    // Visualization function
    void VisualizePointCloud(
        int width = 800,
        int height = 600,
        float scale = 100.0f,
        float offset_x = 400.0f,
        float offset_y = 300.0f
    );
    
    // Getter functions
    const std::vector<std::array<float, 3>>& GetAllWallLines() const;
    const std::array<float, 3>& GetClosestWallLine() const;
    const TimedPointCloud& GetWallPoints() const;
    const TimedPointCloud& GetNoisePoints() const;
    
    // Utility function to check if a point is on a wall
    static bool IsWallPoint(const std::array<float, 3>& wall_line, float obs_x, float obs_y, float threshold = 0.2f);
    
    // Clear detected walls
    void ClearWalls();
};

#endif // WALL_DETECTOR_H