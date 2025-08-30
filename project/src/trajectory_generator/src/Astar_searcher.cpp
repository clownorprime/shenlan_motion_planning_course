#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  GridNodeMap = new GridNodePtr **[GLX_SIZE];
  for (int i = 0; i < GLX_SIZE; i++) {
    GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; j++) {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
      for (int k = 0; k < GLZ_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
        resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
      idx_y == GLY_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
  else {
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
  }
}

vector<Vector3d> AstarPathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (GridNodeMap[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                          vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i neighborIdx;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      for (int dz = -1; dz < 2; dz++) {

        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        neighborIdx(0) = (currentPtr->index)(0) + dx;
        neighborIdx(1) = (currentPtr->index)(1) + dy;
        neighborIdx(2) = (currentPtr->index)(2) + dz;

        if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
            neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
            neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE) {
          continue;
        }

        neighborPtrSets.push_back(
            GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
        edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
      }
    }
  }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  // using digonal distance and one type of tie_breaker.
  double h;
  double x_diff = std::abs(node1->index[0] - node2->index[0]);
  double y_diff = std::abs(node1->index[1] - node2->index[1]);
  double z_diff = std::abs(node1->index[2] - node2->index[2]);
  h = x_diff + y_diff + z_diff + (std::sqrt(2) - 2) * std::min(x_diff, std::min(y_diff, z_diff));
  return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();

  // index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;

  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr = new GridNode(end_idx, end_pt);

  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0;
  /**
   *
   * STEP 1.1:  finish the AstarPathFinder::getHeu
   *
   * **/
  startPtr->fScore = getHeu(startPtr, endPtr);

  startPtr->id = 1;
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));

  /**
   *
   * STEP 1.2:  some else preparatory works which should be done before while
   * loop
   *
   * **/
  for (int i = 0; i < GLX_SIZE; i++)
  {
    for (int j = 0; j < GLY_SIZE; j++)
    {
      for (int k = 0; k < GLZ_SIZE; k++)
      {
        GridNodeMap[i][j][k]->id = 0;
        GridNodeMap[i][j][k]->gScore = inf;
      }
    }
  }
  GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]] = startPtr;
  GridNodeMap[end_idx[0]][end_idx[1]][end_idx[2]] = endPtr;

  double tentative_gScore;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  /**
   *
   * STEP 1.3:  finish the loop
   *
   * **/
  std::cout << "start pt: " << start_pt[0] << " " << start_pt[1] << " " << start_pt[2] << std::endl;
  std::cout << "target pt: " << end_pt[0] << " " << end_pt[1] << " " << end_pt[2] << std::endl;
  while (!openSet.empty()) {
    GridNodePtr node_ptr = openSet.begin()->second;
    openSet.erase(openSet.begin());
    //set the node to be expanded.
    int x_index = node_ptr->index[0], y_index = node_ptr->index[1], z_index = node_ptr->index[2];
    GridNodeMap[x_index][y_index][z_index]->id = -1;
    AstarGetSucc(node_ptr, neighborPtrSets, edgeCostSets);
    for (int i = 0; i < neighborPtrSets.size(); i++)
    {
      GridNodePtr neighbor_ptr = neighborPtrSets[i];
      //neighbor node has been occupied by obstacle.
      if (isOccupied(neighbor_ptr->index))
      {
        continue;
      }
      int x_index = neighbor_ptr->index[0], y_index = neighbor_ptr->index[1], z_index = neighbor_ptr->index[2];
      if (GridNodeMap[x_index][y_index][z_index]->id == 0)
      {
        neighbor_ptr->gScore = node_ptr->gScore + edgeCostSets[i];
        neighbor_ptr->fScore = neighbor_ptr->gScore + getHeu(neighbor_ptr, endPtr);
        neighbor_ptr->nodeMapIt = openSet.insert(make_pair(neighbor_ptr->fScore, neighbor_ptr));
        //mark the neighbor node has already been in open set.
        neighbor_ptr->id = 1;
        neighbor_ptr->cameFrom = node_ptr;
      }
      else if (GridNodeMap[x_index][y_index][z_index]->id == 1)
      {
        if (neighbor_ptr->gScore > node_ptr->gScore + edgeCostSets[i])
        {
          openSet.erase(neighbor_ptr->nodeMapIt);
          neighbor_ptr->gScore = node_ptr->gScore + edgeCostSets[i];
          neighbor_ptr->cameFrom = node_ptr;
          neighbor_ptr->fScore = neighbor_ptr->gScore + getHeu(neighbor_ptr, endPtr);
          neighbor_ptr->nodeMapIt = openSet.insert(make_pair(neighbor_ptr->fScore, neighbor_ptr));
        }
      }
    }

    if (node_ptr == endPtr)
    {
      std::cout << "A* seach success." << std::endl;
      break;
    }
  }

  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
}

vector<Vector3d> AstarPathFinder::getPath() {
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;

  /**
   *
   * STEP 1.4:  trace back the found path
   *
   * **/
  GridNodePtr goal_ptr = GridNodeMap[goalIdx[0]][goalIdx[1]][goalIdx[2]];
  GridNodePtr temp_ptr = goal_ptr;
  while(temp_ptr)
  {
    gridPath.push_back(temp_ptr);
    temp_ptr = temp_ptr->cameFrom;
  }
  int size = gridPath.size();
  for (int i = 0; i < size; i++)
  {
    path.push_back(gridPath[size - i - 1]->coord);
  }

  return path;
}

vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {
  vector<Vector3d> subPath;
  /**
   *
   * STEP 2.1:  implement the RDP algorithm
   *
   * **/
  //如果使用RDP算法简化出的point大小小于3,会影响trajectory generation，所以我们这里对于path size小于3的path不进行优化.
  if (path.size() <= 3)
  {
    return path;
  }
  auto getNodeDistance = [](Vector3d point, Vector3d linePoint1, Vector3d linePoint2) -> double {
    Eigen::Vector3d lineDirection = linePoint2 - linePoint1;
    
    // 计算从直线起点到目标点的向量
    Eigen::Vector3d pointToLineStart = point - linePoint1;
    
    // 计算方向向量的模长平方
    double lineLengthSquared = lineDirection.squaredNorm();
    
    // 如果直线长度为0（两点重合），则直接返回两点之间的距离
    if (lineLengthSquared < 1e-10) {
        return pointToLineStart.norm();
    }
    
    // 计算投影参数t
    double t = pointToLineStart.dot(lineDirection) / lineLengthSquared;
    
    // 计算投影点坐标
    Eigen::Vector3d projectionPoint = linePoint1 + t * lineDirection;
    
    // 返回点到投影点的距离
    return (point - projectionPoint).norm();
  };

  int max_index = -1;
  double max_distance = -1;
  for(int i = 1; i < path.size() - 1; i++)
  {
    double d = getNodeDistance(path[i], path[0], path[path.size() - 1]);
    if (max_distance < d)
    {
      max_distance = d;
      max_index = i;
    }
  }

  if (max_index == -1)
  {
    std::cout << "max_index is -1." << std::endl;
    std::cout << "path: " << std::endl;
    for (int i = 0; i < path.size(); i++)
    {
      std::cout << path[i][0] << " " << path[i][1] << " " << path[i][2] << std::endl;
    }
    return path;
  }

  std::cout << "path size: " << path.size() << std::endl;
  std::cout << "max index: " << max_index << std::endl;
  std::cout << "max distance: " << max_distance << std::endl;
  if (max_distance > path_resolution)
  {
    vector<Vector3d> front_path(path.begin(), path.begin() + max_index);
    vector<Vector3d> back_path(path.begin() + max_index, path.end());
    vector<Vector3d> subPath1 = pathSimplify(front_path, path_resolution);
    vector<Vector3d> subPath2 = pathSimplify(back_path, path_resolution);
    subPath = subPath1;
    subPath.insert(subPath.end(), subPath2.begin(), subPath2.end());
  }
  else
  {
    std::cout << "max_distance is smaller than path_resolution." << std::endl;
    std::cout << "path: " << std::endl;
    for (int i = 0; i < path.size(); i++)
    {
      std::cout << path[i][0] << " " << path[i][1] << " " << path[i][2] << std::endl;
    }
    subPath = {path[0], path[path.size() - 1]};
  }
  return subPath;
}

Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
  /**
   *
   * STEP 3.3:  finish the sareCheck()
   *
   * **/

  return unsafe_segment;
}