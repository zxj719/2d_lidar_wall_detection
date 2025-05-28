// 使用示例 (sdk_main.cpp)
#include "wall_detector.h"

// 全局或类成员变量
static WallDetector g_wall_detector;

int main(int argc, char* argv[]) {
    // ... 其他初始化代码
    
    // 假设 point_cloud.points 和 ROBOT_RADIUS_M 已定义
    // TimedPointCloud point_cloud_points = ...; // 你的点云数据
    // float ROBOT_RADIUS_M = ...; // 机器人半径
    
    // 检测墙壁
    g_wall_detector.DetectWalls(point_cloud_points);
    
    // 找到最近的墙壁
    g_wall_detector.FindClosestWall(ROBOT_RADIUS_M);
    
    // 可视化结果（调试用）
    g_wall_detector.VisualizePointCloud();
    
    // 获取结果
    const auto& all_walls = g_wall_detector.GetAllWallLines();
    const auto& closest_wall = g_wall_detector.GetClosestWallLine();
    const auto& wall_points = g_wall_detector.GetWallPoints();
    const auto& noise_points = g_wall_detector.GetNoisePoints();
    
    // 打印调试信息
    std::cout << "Detected " << all_walls.size() << " wall lines" << std::endl;
    std::cout << "Wall points: " << wall_points.size() << std::endl;
    std::cout << "Noise points: " << noise_points.size() << std::endl;
    
    // ... 其他处理代码
    
    return 0;
}

// 如果需要外部接口函数
void get_wall_lines(void* wall_lines) {
    const auto& closest_line = g_wall_detector.GetClosestWallLine();
    memcpy((std::array<float, 3>*) wall_lines, closest_line.data(), sizeof(std::array<float, 3>));
}