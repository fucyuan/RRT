#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>

using namespace std;

// 定义二维点的结构体
struct Point {
    double x;
    double y;
};

// 计算两个点之间的欧几里得距离
double distance(const Point& a, const Point& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// 生成随机点
Point generateRandomPoint(double maxX, double maxY) {
    Point randomPoint;
    randomPoint.x = (double)rand() / RAND_MAX * maxX;
    randomPoint.y = (double)rand() / RAND_MAX * maxY;
    return randomPoint;
}

// RRT算法函数
vector<Point> rrt(Point start, Point goal, double maxX, double maxY, int maxIter) {
    vector<Point> tree;
    tree.push_back(start);

    for (int i = 0; i < maxIter; ++i)
     {
        Point randomPoint = generateRandomPoint(maxX, maxY);

        // 找到树中最近的点
        int nearestIndex = 0;
        double minDist = distance(tree[0], randomPoint);
        for (int j = 1; j < tree.size(); ++j) 
        {
            double dist = distance(tree[j], randomPoint);
            if (dist < minDist) 
            {
                minDist = dist;
                nearestIndex = j;
            }
        }
        Point nearestPoint = tree[nearestIndex];

        // 将最近的点朝着随机点移动一步
        // 实际中可能需要更复杂的运动模型来实现此步骤
        Point newPoint;
        newPoint.x = nearestPoint.x + ((randomPoint.x - nearestPoint.x) / minDist) * 0.1;
        newPoint.y = nearestPoint.y + ((randomPoint.y - nearestPoint.y) / minDist) * 0.1;

        tree.push_back(newPoint);

        // 如果新点接近目标点，结束搜索
        if (distance(newPoint, goal) < 0.1) {
            tree.push_back(goal);
            break;
        }
    }

    return tree;
}

int main() 
{
    srand(time(0));

    Point start = {1.0, 1.0};
    Point goal = {9.0, 9.0};
    double maxX = 10.0;
    double maxY = 10.0;
    int maxIter = 1000;

    vector<Point> path = rrt(start, goal, maxX, maxY, maxIter);

    cout << "Generated path:" << endl;
    for (const Point& point : path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    return 0;
}
