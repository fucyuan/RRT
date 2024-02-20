# RRT学习

https://zhuanlan.zhihu.com/p/133224593
## 基础RRT
RRT（快速探索随机树）算法是一种用于路径规划的算法，其基本流程如下：
![Alt text](image-9.png)
在这段代码中，M是地图的数据结构，x_init是起始点，x_goal是目标点。随机在地图中的可行区域生成随机节点x_rand；然后查找最近点x_near，即在当前的地图中与x_rand距离最近的点；然后沿着x_rand和x_near之间的stepsize方向尝试延伸新的节点x_new；CollisionFree(M, E)是检查新生成的边(x_new, x_near)是否与地图中的障碍物没有冲突，如果没有冲突，则将x_new加入到地图中成为一个新的节点并连接E。重复上述过程，直至找到从起点到终点的路径。
![Alt text](image-12.png)
1. **初始化**：
   - 确定搜索空间（通常是一个包含机器人或车辆可行动区域的区域）。
   - 指定起始点 `x_init` 和目标点 `x_goal`。
   - 创建一棵树 `T`，并将起始点作为树的根节点。

2. **迭代**：
   - 从搜索空间中随机采样一个点 `x_rand`。
   - 在树 `T` 中找到离 `x_rand` 最近的节点 `x_near`。
   - 从节点 `x_near` 沿着一定步长（StepSize）朝向 `x_rand` 移动，生成一个新的节点 `x_new`。
   - 如果从 `x_near` 到 `x_new` 的路径上没有碰撞（即路径是自由的），则将 `x_new` 加入树 `T`，并以 `x_near` 到 `x_new` 的边作为树的新分支。
   - 重复以上步骤直到达到终止条件：
     - 当找到一个新的节点 `x_new` 和目标点 `x_goal` 的距离小于某个阈值时，或者树 `T` 中节点的数量达到一定的限制。
     - 或者可以通过其他条件判断搜索是否结束。
![Alt text](image-13.png)
3. **连接到目标**：
   - 如果最后生成的节点 `x_new` 足够接近目标点 `x_goal`，则连接 `x_new` 到 `x_goal`，完成路径。

4. **路径提取**：
   - 从树 `T` 中回溯从起始点到目标点的路径。这可以通过从目标节点开始回溯，沿着树的边找到父节点，直到回到起始点为止。

5. **路径优化**（可选）：
   - 对提取的路径进行优化，以满足特定的约束条件或者最小化某种指标（例如路径长度、平滑度等）。

通过这样的流程，RRT算法能够在高维、复杂的空间中快速生成可行的路径。
### C++代码简单实现
```cpp
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

    for (int i = 0; i < maxIter; ++i) {
        Point randomPoint = generateRandomPoint(maxX, maxY);

        // 找到树中最近的点
        int nearestIndex = 0;
        double minDist = distance(tree[0], randomPoint);
        for (int j = 1; j < tree.size(); ++j) {
            double dist = distance(tree[j], randomPoint);
            if (dist < minDist) {
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

int main() {
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
```
## 效率优化 RRT
### KD-tree 运用在RRT中

KD树（k维树）是一种在多维空间中组织点的数据结构，用于支持快速检索操作。在RRT（Rapidly-exploring Random Tree，快速随机树）算法中，KD树可以有效地用于找到最近邻点，即在树中找到离新采样点最近的点（`x_near`）。

在RRT算法中，每次迭代都需要找到距离新随机采样点（`x_rand`）最近的树节点（`x_near`），以便从该节点向采样点方向扩展新的分支。在高维空间中，如果使用简单的线性搜索，这个过程可能非常耗时，因为需要计算新采样点与树中每一个点的距离。这是KD树发挥作用的地方。

在RRT中使用KD树的基本步骤如下：

1. **初始化**：在算法开始时，初始化一个KD树，包含起始点`x_init`。

2. **添加节点**：当算法向RRT添加新节点`x_new`时，同时将这个新节点添加到KD树中。

3. **查找最近邻**：每次需要找到新的随机采样点`x_rand`的最近邻时，使用KD树来快速检索。

4. **优化**：由于KD树的结构，查找最近邻的时间复杂度通常比线性搜索要低，特别是在点的数量较多时。

5. **更新**：每次树T添加新的节点后，KD树也需要更新，以包含新的点。

![Alt text](image-20.png)
![Alt text](image-21.png)
KD树优化了查找最近邻的过程，允许RRT算法在较高维度的空间中更有效地运行。尽管如此，KD树本身在每次插入新节点时也需要维护，特别是在动态环境中，这可能会带来额外的计算负担。此外，KD树的效率可能会随着维度的增加而降低，这是所谓的“维度的诅咒”。尽管存在这些潜在的问题，KD树通常仍然是提高RRT性能的有效方法。
### Bidirectional RRT / RRT Connect
RRT Connect算法从初始状态点和目标状态点同时扩展随机树从而实现对状态空间的快速搜索。

![Alt text](image-19.png)
RRT-Connect算法是RRT（Rapidly-exploring Random Tree）算法的一个变体，它以更高的效率寻找从起始状态到目标状态的路径。RRT-Connect从起始点和目标点同时构建两棵树（称为`Ta`和`Tb`），一棵从起始点开始生长，另一棵从目标点开始生长，并尝试连接两棵树。当两棵树相遇时，路径就被找到了。

RRT-Connect算法的关键实现步骤是：

1. **双树搜索**：同时从起始点和目标点构建两棵树。
2. **连接尝试**：每次迭代中，尝试将一棵树扩展到另一棵树中的点。
3. **交替扩展**：交替地扩展两棵树，这有助于平衡搜索过程并避免一棵树过于集中在状态空间的一个区域。
4. **路径平滑**：一旦两棵树连接，可以通过移除多余的节点和平滑操作来优化路径。

下面是RRT-Connect算法的伪代码

```cpp
算法：RRT-Connect
输入：x_init（初始点）, x_goal（目标点）
输出：从x_init到x_goal的路径

过程 RRT-Connect(x_init, x_goal)
  Ta.init(x_init)  // 初始化树Ta，根为初始点
  Tb.init(x_goal)  // 初始化树Tb，根为目标点
  while true do
    x_rand ← Sample()  // 随机采样一个点x_rand
    if Extend(Ta, x_rand) ≠ Trapped then  // 尝试向x_rand扩展树Ta
      if Extend(Tb, Ta.newest) = Reached then  // 如果树Ta可以扩展到树Tb的最新点，则路径找到
        return Path(Ta, Tb)  // 返回从Ta到Tb的路径
    Swap(Ta, Tb)  // 交换Ta和Tb，下一次迭代将尝试扩展Tb
  end while
End 过程

函数 Extend(T, x)
  x_near ← NearestNeighbor(T, x)  // 在树T中找到最接近x的点x_near
  x_new ← Steer(x_near, x)  // 从x_near朝x方向生成一个新点x_new
  if CollisionFree(x_near, x_new) then  // 检查x_near到x_new的路径是否无碰撞
    T.add(x_new)  // 如果无碰撞，将x_new添加到树T
    if x_new = x then
      return Reached  // 如果x_new等于x，则表示达到了x
    else
      return Advanced  // 否则，表示树向x方向前进了一步
  end if
  return Trapped  // 如果从x_near到x_new有碰撞，表示这个方向走不通
End 函数
```
在这个伪代码中：

- `Sample()`随机在状态空间中选取一个点。
- `Extend()`函数尝试将树向`x_rand`扩展，如果新点`x_new`可以无障碍地连接到`x_near`，则将它添加到树中。
  - 如果`x_new`与`x_rand`一样，我们认为已经达到了目标。
  - 如果我们能够向`x_rand`方向前进，即使没有到达，也表示有进展。
  - 如果不能前进，表示陷入了障碍物或者走不动了。
- `NearestNeighbor()`函数在树`T`中找到最接近`x`的点。
- `Steer()`函数从`x_near`朝向`x`方向生成新点`x_new`。
- `CollisionFree()`函数检查从`x_near`到`x_new`的路径是否没有与障碍物碰撞。
- `Swap()`函数交换两棵树，以便交替扩展。

RRT-Connect的效率较高，因为它减少了搜索空间，同时扩展两棵树加速了搜索过程，通常比单树RRT更快找到路径。
