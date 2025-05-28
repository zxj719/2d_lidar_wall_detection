#include "wall_detector.h"

using namespace cv;

WallDetector::WallDetector() {
    m_wall_closest_line = {{0.0f, 20.0f, 0.0f}};
}

float WallDetector::PointDistance(const Eigen::Vector4f& p1, const Eigen::Vector4f& p2) {
    return std::sqrt(
        (p1[0] - p2[0]) * (p1[0] - p2[0]) + 
        (p1[1] - p2[1]) * (p1[1] - p2[1])
    );
}

std::pair<Eigen::Vector2f, float> WallDetector::LeastSquaresFit(const Eigen::MatrixXf& points) {
    int n_points = points.rows();
    Eigen::VectorXf y = points.col(0);
    Eigen::VectorXf x = points.col(1);
    
    // Create A matrix for least squares fitting [x, ones]
    Eigen::MatrixXf A(n_points, 2);
    A.col(0) = y;
    A.col(1) = Eigen::VectorXf::Ones(n_points);
    
    // Solve for coefficients
    Eigen::Vector2f coeffs = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(x);
    
    // Calculate fitted y values
    Eigen::VectorXf x_fit = A * coeffs;
    
    // Calculate variance
    float var = (x - x_fit).squaredNorm() / n_points;
    
    return {coeffs, var};
}

float WallDetector::PointFitVar(const Eigen::Vector2f& point, const Eigen::Vector2f& coeffs) {
    float predicted_x = point[1] * coeffs[0] + coeffs[1];
    float residual = predicted_x - point[0];
    return residual * residual;
}

void WallDetector::DetectWalls(
    const TimedPointCloud& points,
    float max_fit_var,
    float min_span_distance,
    float min_distance,
    float max_distance,
    int segment_window
) {
    // Clear previous results
    m_wall_lines.clear();
    m_wall_points.clear();
    m_noise_points.clear();

    TimedPointCloud filtered_points;
    Eigen::Vector4f last_point = {0.0f, 0.0f, 0.0f, 0.0f};
    for (const auto& pt : points) {
        if (!(pt[0] == 0.0f && pt[1] == 0.0f) && PointDistance(pt, last_point) > min_distance) {
            filtered_points.push_back(pt);
            last_point = pt;
        }
    }

    int N = filtered_points.size();
    if (N == 0) return;

    int offset = static_cast<int>(N/2);
    TimedPointCloud working_points(N);
    for (int i = 0; i < N; ++i) {
        working_points[i] = filtered_points[(i + offset) % N];
    }
    
    // Project 3D points to 2D for line fitting (using x and y coordinates)
    Eigen::MatrixXf working_data(N, 2);
    for (int i = 0; i < N; ++i) {
        working_data(i, 1) = working_points[i][0]; // x
        working_data(i, 0) = working_points[i][1]; // y
    }
    
    // Forward pass
    std::vector<bool> wall_mask(N, false);
    
    int i = 0;
    while (i < N - segment_window + 1) {
        // Check distance of a bunch for continuity
        int j = i + segment_window - 1;
        if (j >= N) break;
        bool flag = false;
        for(int k = i; k < j; k++){
            if(PointDistance(working_points[k], working_points[k+1]) > max_distance){
                i = k+1;
                flag = true;
                break;
            }
        }
        if(flag){
            continue;
        }
        
        // Extract segment
        Eigen::MatrixXf segment = working_data.block(i, 0, j - i + 1, 2);
        
        float var;
        Eigen::Vector2f coeffs;
        // Fit line to segment
        std::tie(coeffs, var) = LeastSquaresFit(segment);
        
        if (var < max_fit_var) {
            // Try extending the segment from both sides
            int count = 0;
            int factor = segment_window;
            while (j + 1 < N) {
                // Check distance before extending
                float dist = PointDistance(working_points[j], working_points[j+1]);
                if (dist > max_distance) {
                    break; // Stop extending if points are too far apart
                }
                
                float new_var = PointFitVar(working_data.row(j+1), coeffs);
                
                if (new_var < factor * max_fit_var) {
                    j++;
                    count++;
                    factor = factor > 1 ? factor - 1 : 1;
                    if(count == segment_window){
                        std::tie(coeffs, std::ignore) = LeastSquaresFit(working_data.block(i, 0, j - 1 + 1, 2));
                    }
                } else {
                    break;
                }
            }

            int i_extended = i;
            count = 0;
            factor = segment_window;
            while (i_extended > 0) {
                // Check distance before extending
                float dist = PointDistance(working_points[i_extended], working_points[i_extended-1]);
                if (dist > max_distance) {
                    break; // Stop extending if points are too far apart
                }
                
                float new_var = PointFitVar(working_data.row(i_extended-1), coeffs);
                
                if (new_var < max_fit_var) {
                    i_extended--;
                    count++;
                    factor = factor > 1 ? factor - 1 : 1;
                    if(count == segment_window){
                        std::tie(coeffs, std::ignore) = LeastSquaresFit(working_data.block(i_extended, 0, j - i_extended + 1, 2));
                    }
                } else {
                    break;
                }
            }
            
            // Mark segment as wall
            if (PointDistance(working_points[j], working_points[i_extended]) > min_span_distance){
                for (int k = i_extended; k <= j && k < N; ++k) {
                    wall_mask[k] = true;
                }
                std::tie(coeffs, var) = LeastSquaresFit(working_data.block(i_extended, 0, j - i_extended + 1, 2));
                m_wall_lines.push_back({coeffs[0], coeffs[1], var});
                i = j + 1;
            } else {
                i = i + 10;
            }
        } else {
            i += 1;
        }
    }
    
    // Separate points into wall and noise point clouds
    for (int i = 0; i < N; ++i) {
        if (wall_mask[i]) {
            m_wall_points.push_back(working_points[i]);
        } else {
            m_noise_points.push_back(working_points[i]);
        }
    }
}

void WallDetector::FindClosestWall(float robot_radius) {
    float wall_min_dist = 20.0f;
    std::array<float, 3> wall_closest_line = {{0.0f, 20.0f, 0.0f}};
    
    for(const auto& line : m_wall_lines) {
        float distance = std::abs(-robot_radius + line[1]) / std::sqrt(line[0] * line[0] + 1.0f);
        if(distance < wall_min_dist) {
            wall_min_dist = distance;
            wall_closest_line = line;
        }
    }
    
    m_wall_closest_line = wall_closest_line;
}

void WallDetector::VisualizePointCloud(
    int width,
    int height, 
    float scale,
    float offset_x,
    float offset_y
) {
    // Create a blank image with white background
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    
    // Function to map from point cloud coordinates to image coordinates
    auto mapToImage = [scale, offset_x, offset_y](const Eigen::Vector4f& point) -> cv::Point {
        return cv::Point(
            static_cast<int>(-point[0] * scale + offset_x), // 根据数据特征水平翻转
            static_cast<int>(point[1] * scale + offset_y)
        );
    };
    
    // Draw origin (robot position)
    cv::circle(image, cv::Point(offset_x, offset_y), 4, cv::Scalar(0, 0, 255), -1);

    // Draw noise points in blue
    for (const auto& point : m_noise_points) {
        cv::circle(image, mapToImage(point), 2, cv::Scalar(255, 0, 0), -1);
    }
    
    // Draw wall points in green
    for (const auto& point : m_wall_points) {
        cv::circle(image, mapToImage(point), 2, cv::Scalar(0, 255, 0), -1);
    }
    
    // Draw fitted lines
    for(const auto& seg : m_wall_lines){
        float a = seg[0], b = seg[1];
        float y1 = -offset_y / scale;
        float x1 = a * y1 + b;
        float y2 = offset_y / scale;
        float x2 = a * y2 + b;
        cv::line(image, mapToImage(Eigen::Vector4f(x1, y1, 0, 0)), 
                 mapToImage(Eigen::Vector4f(x2, y2, 0, 0)), cv::Scalar(0, 0, 255), 2);
    }
    
    // Add legend
    cv::rectangle(image, cv::Point(20, 20), cv::Point(40, 40), cv::Scalar(0, 255, 0), -1);
    cv::putText(image, "Wall Points", cv::Point(50, 35), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    cv::rectangle(image, cv::Point(20, 50), cv::Point(40, 70), cv::Scalar(255, 0, 0), -1);
    cv::putText(image, "Noise Points", cv::Point(50, 65), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

    cv::rectangle(image, cv::Point(20, 80), cv::Point(40, 100), cv::Scalar(0, 0, 255), -1);
    cv::putText(image, "Fitted Lines", cv::Point(50, 95), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    // Display statistics
    std::string stats = "Wall points: " + std::to_string(m_wall_points.size()) + 
                        ", Noise points: " + std::to_string(m_noise_points.size()) +
                        ", Lines: " + std::to_string(m_wall_lines.size());
    cv::putText(image, stats, cv::Point(20, height - 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    // Show the image
    cv::imshow("Point Cloud Visualization", image);
    cv::waitKey(1);
}

const std::vector<std::array<float, 3>>& WallDetector::GetAllWallLines() const {
    return m_wall_lines;
}

const std::array<float, 3>& WallDetector::GetClosestWallLine() const {
    return m_wall_closest_line;
}

const TimedPointCloud& WallDetector::GetWallPoints() const {
    return m_wall_points;
}

const TimedPointCloud& WallDetector::GetNoisePoints() const {
    return m_noise_points;
}

bool WallDetector::IsWallPoint(const std::array<float, 3>& wall_line, float obs_x, float obs_y, float threshold) {
    float distance = std::abs(wall_line[0] * obs_y - obs_x + wall_line[1]) / std::sqrt(wall_line[0] * wall_line[0] + 1.0f);
    return distance < threshold;
}

void WallDetector::ClearWalls() {
    m_wall_lines.clear();
    m_wall_points.clear();
    m_noise_points.clear();
    m_wall_closest_line = {{0.0f, 20.0f, 0.0f}};
}