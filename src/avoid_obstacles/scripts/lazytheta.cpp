#include <iostream>

// #define USE_PCL

// Memory structures
#include <string>
#include <set>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <memory>

#ifdef USE_PCL
#include <pcl/point_cloud.h>
#endif

// Math-related
#include <limits>
#include <cmath>
#include <eigen3/Eigen/Core>
using namespace std;
using namespace Eigen;

// Converting Vector3i into string courtesy of:
// https://stackoverflow.com/a/50397167
static string toString(const MatrixXi& mat){
    stringstream ss;
    ss << mat;
    return ss.str();
}

// Node/vertex class
// Position coordinates (x, y, z): integers in Vector3i
// Scores (gScore, fScore): float/float64
struct Node {
    Vector3i xyz;
    float gScore = 0, hScore = 0, fScore = 0;
    Node* parent = nullptr;
	
	void UpdateGScore(float p_gScore = numeric_limits<float>::max()) {
		gScore = p_gScore;
		fScore = gScore + hScore;
	}

	void UpdateHScore(Node* goal) {
		hScore = sqrt((xyz - goal->xyz).squaredNorm());
	}

	// Constructors
	// With gScore, goal and parent
    Node(Vector3i p_xyz, float p_gScore, Node* goal, Node* p_parent) {
		xyz = p_xyz;
		gScore = p_gScore;
		hScore = 0;
		fScore = gScore;
		parent = p_parent;
		if (goal != nullptr) 
			UpdateHScore(goal);
    }
	// With gScore & goal
    Node(Vector3i p_xyz, float p_gScore, Node* goal) {
		xyz = p_xyz;
		gScore = p_gScore;
		hScore = 0;
		fScore = gScore;
		parent = nullptr;
		if (goal != nullptr) 
			UpdateHScore(goal);
    }
	// With goal
    Node(Vector3i p_xyz, Node* goal) {
		xyz = p_xyz;
		gScore = numeric_limits<float>::max();
		hScore = 0;
		fScore = gScore;
		if (goal != nullptr) 
			UpdateHScore(goal);
    }
	// Default
	Node(Vector3i p_xyz = Vector3i(0, 0, 0)) {
		xyz = p_xyz;
		gScore = numeric_limits<float>::max();
		hScore = 0;
		fScore = gScore;
		parent = nullptr;
	}

	bool operator<(const Node& other) const {
		return fScore < other.fScore;
	}
};

// Struct to compare Node class pointers courtesy of:
// http://www.java2s.com/ref/cpp/cpp-multiset-automatically-sort-person-objects-stored-by-pointer.html
struct compareNodePtr {
	bool operator()(const Node* s1, const Node* s2) const {
		return s1->fScore < s2->fScore;
	}
};

// Vector3i hash courtesy of:
// https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
// Hash function for Eigen matrix and vector.
// The code is from `hash_combine` function of the Boost library. See
// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    // Note that it is oblivious to the storage order of Eigen matrix (column- or
    // row-major). It will give you the same hash value for two different matrices if they
    // are the transpose of each other in different storage order.
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// Define some types to type less
typedef unordered_set<Vector3i, matrix_hash<Vector3i>> 							Vec3iu_set;
typedef unordered_set<Vector3i, matrix_hash<Vector3i>>::const_iterator			Vec3iu_set_ci;
typedef unordered_set<Vector3i, matrix_hash<Vector3i>>::iterator				Vec3iu_set_i;
typedef unordered_map<Vector3i, Node*, matrix_hash<Vector3i>> 					Vec3iu_map;
typedef unordered_map<Vector3i, Node*, matrix_hash<Vector3i>>::const_iterator	Vec3iu_map_ci;
typedef unordered_map<Vector3i, Node*, matrix_hash<Vector3i>>::iterator			Vec3iu_map_i;
typedef multiset<Node*, compareNodePtr>											Nodemlt_set;															

// Define sign function
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


// LazyTheta class
struct LazyTheta {
	// Dynamically allocated vector of Nodes
	vector<unique_ptr<Node>> nodes;

	// To-explore stack
	Vec3iu_map open;
	// To-explore stack as an fScore sorted set
	multiset<Node*, compareNodePtr> openList;
	// To-explore xyz as unordered set (for quick neighbor lookup)
	// Vec3iu_set openXYZ;

	// Explored stack
	Vec3iu_map closed;
	// Explored xyz as unordered set (for quick neighbor lookup)
	// Vec3iu_set closedXYZ;

	// Blocked grid in form of set of tuples (most efficient)
	Vec3iu_set blockedXYZ;
	// Set of registered central obstacle cells
	Vec3iu_set obstaclesXYZ;
	// Get water surface z
	float zMax = zMax;
	// Get water depth z
	float zMin = zMin;

	// Unit dimensions of occupancy grid (meters)
	float OCC_UNIT_X = 0.1, OCC_UNIT_Y = 0.1, OCC_UNIT_Z = 0.1, OCC_UNIT = 0.1; // meters
	// Determining characteristics of obstacle definition
	int OBSTACLE_THRESHOLD = 255;
	// Define boundaries for obstacles (truncated box)
	float AVOIDANCE_RADIUS = 0.5; // meters
	int BOUND = (int)round(AVOIDANCE_RADIUS / OCC_UNIT);
	float BOUND_X = BOUND, BOUND_Y = BOUND, BOUND_Z = BOUND;
	// Get costs of neighbors
	float NGBR_COST[3] = {1, 1.41421, 1.73205};


	// Placeholder start & end node
	Node s_start;
	Node s_end;

	// Default constructor
	LazyTheta() {}

	// Create boundary cuboids
	void AddObstacle(const Vector3i& xyz) {
		// Hard-coding the matrix courtesy of:
		// https://stackoverflow.com/a/35170514
		static Matrix<int, 3, 254> result, const_obs = (Matrix<int, 3, 254>() << -4, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, -3, -3, -3, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, -4, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, -3, -3, -3, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 0, -1, 0, 1, -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -1, 0, 1, -1, 0, 1, -2, -1, 0, 1, 2, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -2, -1, 0, 1, 2, -1, 0, 1, -2, -1, 0, 1, 2, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -2, -1, 0, 1, 2, 0, -2, -1, 0, 1, 2, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -4, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -2, -1, 0, 1, 2, -1, 0, 1, -2, -1, 0, 1, 2, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -3, -2, -1, 0, 1, 2, 3, -2, -1, 0, 1, 2, -1, 0, 1, -1, 0, 1, -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -1, 0, 1).finished();
		// Adding vector to matrix courtesy of:
		// https://stackoverflow.com/a/42811594
		// https://stackoverflow.com/a/16287999
		result = const_obs.colwise() + xyz;
		for (size_t i = 0, size = result.size() / 3; i < size; i++)
			blockedXYZ.emplace(*(result.data() + i * 3), *(result.data() + i * 3 + 1), *(result.data() + i * 3 + 2));
	}

	// Update occupancy set iteratively
	inline void UpdateOccupancy(const Vector3i& occ) {
		auto result = obstaclesXYZ.emplace(occ);
		if (result.second) AddObstacle(occ);
	}

	// Update occupancy set, with updateStart and updateBox vectors
	void UpdateOccupancySet(const vector<Vector3i>& occSet,
							bool update = false, bool bounds = false,
							Vector3i updateStart = Vector3i(0, 0, 0),
							Vector3i updateBox = Vector3i(0, 0, 0)) {
		if (!update) {
			obstaclesXYZ = Vec3iu_set{};
			blockedXYZ = Vec3iu_set{};
			for (Vector3i i: occSet)
				UpdateOccupancy(i);
		} else {
			if (!bounds)
				for (Vector3i i: occSet)
				UpdateOccupancy(i);
			else {
				Vector3i updateEnd = updateStart + updateBox;
				bool inRange = true;
				for (Vector3i i: occSet) {
					inRange = true;
					for (int j = 0; j < 3; j++)
						if (updateStart(j) > i(j) || i(j) >= updateEnd(j))
							inRange = false;
					if (inRange)
						UpdateOccupancy(i);
				}
			}
		}
	}

#ifdef USE_PCL
	// Update occupancy set using PCL, with updateStart and updateBox vectors
	void UpdateOccupancyPCL(const pcl::PointCloud<pcl::PointXYZ>& occPC,
							bool update = false, bool bounds = false,
							Vector3i updateStart = Vector3i(0, 0, 0),
							Vector3i updateBox = Vector3i(0, 0, 0)) {
		if (!update) {
			obstaclesXYZ = Vec3iu_set{};
			blockedXYZ = Vec3iu_set{};
			for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = occPC.begin(); it != occPC.end(); it++)
				UpdateOccupancy(Vector3i(round(it->x * 10), round(it->y * 10), round(it->z * 10)));
		} else {
			if (!bounds)
				for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = occPC.begin(); it != occPC.end(); it++)
					UpdateOccupancy(Vector3i(round(it->x * 10), round(it->y * 10), round(it->z * 10)));
			else {
				Vector3i updateEnd = updateStart + updateBox;
				bool inRange = true;
				Vector3i tmp;
				for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = occPC.begin(); it != occPC.end(); it++) {
					tmp = Vector3i(round(it->x * 10), round(it->y * 10), round(it->z * 10));
					inRange = true;
					for (int j = 0; j < 3; j++)
						if (updateStart(j) > tmp(j) || tmp(j) >= updateEnd(j))
							inRange = false;
					if (inRange)
						UpdateOccupancy(tmp);
				}
			}
		}
	}
#endif

	// Quickly check neighbors without too many lookups
	vector<pair<Vector3i, float>> GetVisNeighborsXYZ(const Vector3i& xyz) {
		vector<bool> obs;
		for (int x = xyz(0) - 1; x <= xyz(0); x++) 
			for (int y = xyz(1) - 1; y <= xyz(1); y++) 
				for (int z = xyz(2) - 1; z <= xyz(2); z++)
					obs.push_back(blockedXYZ.find(Vector3i(x, y, z)) == blockedXYZ.end());
		
		// Get neighboring nodes
		vector<pair<Vector3i, float>> nbs;
		// Whole plane for direct points
		if (obs[0] & obs[1] & obs[2] & obs[3]) nbs.push_back(make_pair(xyz + Vector3i(-1, 0, 0), NGBR_COST[0]));
		if (obs[4] & obs[5] & obs[6] & obs[7]) nbs.push_back(make_pair(xyz + Vector3i(1, 0, 0), NGBR_COST[0]));
		if (obs[0] & obs[1] & obs[4] & obs[5]) nbs.push_back(make_pair(xyz + Vector3i(0, -1, 0), NGBR_COST[0]));
		if (obs[2] & obs[3] & obs[6] & obs[7]) nbs.push_back(make_pair(xyz + Vector3i(0, 1, 0), NGBR_COST[0]));
		if (obs[0] & obs[2] & obs[4] & obs[6]) nbs.push_back(make_pair(xyz + Vector3i(0, 0, -1), NGBR_COST[0]));
		if (obs[1] & obs[3] & obs[5] & obs[7]) nbs.push_back(make_pair(xyz + Vector3i(0, 0, 1), NGBR_COST[0]));

		// Column for diagonal points
		if (obs[0] & obs[2]) nbs.push_back(make_pair(xyz + Vector3i(-1, 0, -1), NGBR_COST[1]));
		if (obs[4] & obs[6]) nbs.push_back(make_pair(xyz + Vector3i(1, 0, -1), NGBR_COST[1]));
		if (obs[0] & obs[4]) nbs.push_back(make_pair(xyz + Vector3i(0, -1, -1), NGBR_COST[1]));
		if (obs[2] & obs[6]) nbs.push_back(make_pair(xyz + Vector3i(0, 1, -1), NGBR_COST[1]));
		if (obs[0] & obs[1]) nbs.push_back(make_pair(xyz + Vector3i(-1, -1, 0), NGBR_COST[1]));
		if (obs[4] & obs[5]) nbs.push_back(make_pair(xyz + Vector3i(1, -1, 0), NGBR_COST[1]));
		if (obs[2] & obs[3]) nbs.push_back(make_pair(xyz + Vector3i(-1, 1, 0), NGBR_COST[1]));
		if (obs[6] & obs[7]) nbs.push_back(make_pair(xyz + Vector3i(1, 1, 0), NGBR_COST[1]));
		if (obs[1] & obs[3]) nbs.push_back(make_pair(xyz + Vector3i(-1, 0, 1), NGBR_COST[1]));
		if (obs[5] & obs[7]) nbs.push_back(make_pair(xyz + Vector3i(1, 0, 1), NGBR_COST[1]));
		if (obs[1] & obs[5]) nbs.push_back(make_pair(xyz + Vector3i(0, -1, 1), NGBR_COST[1]));
		if (obs[3] & obs[7]) nbs.push_back(make_pair(xyz + Vector3i(0, 1, 1), NGBR_COST[1]));

		// Block for corner points
		if (obs[0]) nbs.push_back(make_pair(xyz + Vector3i(-1, -1, -1), NGBR_COST[2]));
		if (obs[1]) nbs.push_back(make_pair(xyz + Vector3i(-1, -1, 1), NGBR_COST[2]));
		if (obs[2]) nbs.push_back(make_pair(xyz + Vector3i(-1, 1, -1), NGBR_COST[2]));
		if (obs[3]) nbs.push_back(make_pair(xyz + Vector3i(-1, 1, 1), NGBR_COST[2]));
		if (obs[4]) nbs.push_back(make_pair(xyz + Vector3i(1, -1, -1), NGBR_COST[2]));
		if (obs[5]) nbs.push_back(make_pair(xyz + Vector3i(1, -1, 1), NGBR_COST[2]));
		if (obs[6]) nbs.push_back(make_pair(xyz + Vector3i(1, 1, -1), NGBR_COST[2]));
		if (obs[7]) nbs.push_back(make_pair(xyz + Vector3i(1, 1, 1), NGBR_COST[2]));
		// if (obs[]) nbs.push_back(make_pair(xyz + Vector3i(, , ), NGBR_COST[2]));
		return nbs;
	}

	// Implementing fast ray traversal algorithm
    // http://www.cse.yorku.ca/~amana/research/grid.pdf
	bool LineOfSight(const Vector3i& xyzStart, const Vector3i& xyzEnd) {
		// cout << "los " << xyzStart(0) << " " << xyzStart(1) << " " << xyzStart(2) << " " << xyzEnd(0) << " " << xyzEnd(1) << " " << xyzEnd(2) << " " << endl;
		vector<pair<Vector3i, float>> nbs = GetVisNeighborsXYZ(xyzStart);
		for (pair<Vector3i, float> i: nbs)
			if (i.first == xyzEnd)
				return true;
		
		// Ray traversing from end to start
		Vector3i vi = xyzStart - xyzEnd;
		bool aligned[3] = {!(bool)vi(0), !(bool)vi(1), !(bool)vi(2)};
		Vector3i step(sgn(vi(0)), sgn(vi(1)), sgn(vi(2)));
		float norm = sqrt(vi.squaredNorm());
		if (norm < 0.001) return true;
		// Vector3f v(vi(0) / norm, vi(1) / norm, vi(2) / norm);

		// If not aligned on any axis
		if (vi.all()) {
			// All is well
			// Define start & end
			Vector3i startVoxel(xyzEnd + (Vector3i(-1, -1, -1) + step) / 2);
			Vector3i endVoxel(xyzStart + (Vector3i(-1, -1, -1) - step) / 2);
			Vector3f tDelta((float)step(0) / vi(0), (float)step(1) / vi(1), (float)step(2) / vi(2));
			Vector3f tMax = tDelta; // starting from vertex, not crossing any boundaries
			
			// Check for corner moves
			bool doubleMove = false, tripleMove = false;
			Vector2i dMAxes; int dMAx;
			if (vi(0) == vi(1) && vi(1) == vi(2)) {
				tripleMove = true;
			} 
			for (int i = 0; i < 3; i++) {
				if (startVoxel(i) == endVoxel(i)) tMax(i) = numeric_limits<float>::max();
				if (!tripleMove && vi(i) == vi((i + 1) % 3)) {
					if (startVoxel(i) != endVoxel(i) && startVoxel((i + 1) % 3) != endVoxel((i + 1) % 3)) {
						doubleMove = true;
						dMAxes = Vector2i(i, (i + 1) % 3);
						dMAx = (i + 2) % 3;
					}
				}
			}
			// Check if should prioritize double corner moves
			int tDeltaMin = 0;
			float min_tDelta = tDelta.minCoeff(&tDeltaMin);
			bool prioritizeDoubleMove = dMAx != tDeltaMin;

			Vector3i XYZ = startVoxel;
			if (blockedXYZ.find(XYZ) != blockedXYZ.end())
				return false;
			
			if (tripleMove) {
				// bool yes = false;
				// if (xyzEnd == Vector3i(39, 9, 17)) yes = true;
				// if (yes)
				// cout << "triplemove 1 " << XYZ(0) << " " << XYZ(1) << " " << XYZ(2) << " " << endVoxel(0) << " " << endVoxel(1) << " " << endVoxel(2) << endl;
				while (XYZ != endVoxel) {
					// cout << "\t\t " << XYZ(0) << " " << XYZ(1) << " " << XYZ(2) << " " << endl;
					XYZ += step;
					if (blockedXYZ.find(XYZ) != blockedXYZ.end())
						return false;
				}
			} else if (doubleMove) {
				// bool yes = false;
				// if (xyzEnd == Vector3i(39, 9, 17)) yes = true;
				// if (yes)
				// cout << "doublemove 1 " << XYZ(0) << " " << XYZ(1) << " " << XYZ(2) << " " << endVoxel(0) << " " << endVoxel(1) << " " << endVoxel(2) << endl;
				int tMaxMin = 0;
				float min_tMax;
				while (XYZ != endVoxel) {
					// if (XYZ(0) > -100 && XYZ(1) > -100)
					// cout << "\t " << tMax[0] << tMax[1] << XYZ(0) << " " << XYZ(1) << " " << XYZ(2) << " " << endl;
					min_tMax = tMax.minCoeff(&tMaxMin);
					// If the two doubleMove axes are min -> prioritize
					if (dMAx != tMaxMin || (tMax[dMAx] == tMax[dMAxes(0)] && prioritizeDoubleMove)) {
						for (int i = 0; i < 2; i++) {
							XYZ(dMAxes(i)) += step(dMAxes(i));
							tMax(dMAxes(i)) += tDelta(dMAxes(i));
						}
					// Else this is normal
					} else {
						XYZ(dMAx) += step(dMAx);
						tMax(dMAx) += tDelta(dMAx);
					}
					if (blockedXYZ.find(XYZ) != blockedXYZ.end())
						return false;
				}
			} else {
				// bool yes = false;
				// if (xyzEnd == Vector3i(39, 9, 17) || xyzEnd == Vector3i(27, 5, 15)) yes = true;
				// if (yes)
				// cout << "normalmove 1 " << XYZ(0) << " " << XYZ(1) << " " << XYZ(2) << " " << endVoxel(0) << " " << endVoxel(1) << " " << endVoxel(2) << endl;
				Vector3i tDeltaMinDual(0, 0, 0);
				for (int i = 0; i < 3; i++)
					tDeltaMinDual(i) = (tDelta(i) < tDelta((i + 1) % 3))? i: (i + 1) % 3;
				while (XYZ != endVoxel) {
					// if (XYZ(0) > -100 && XYZ(1) > -100)
					// if (yes)
					// cout << "\t\t " << XYZ(0) << " " << XYZ(1) << " " << XYZ(2) << " " << endl; // tMax(0) << tMax(1) << tMax(2) << 
					// If all 3 tMax are equal -> prioritize smallest tDelta
					if (tMax(0) == tMax(1) && tMax(1) == tMax(2)) {
						XYZ(tDeltaMin) += step(tDeltaMin);
						tMax(tDeltaMin) += tDelta(tDeltaMin);
					} else {
						if (tMax(0) < tMax(1)) {
							// Normal
							if (tMax(0) < tMax(2)) {
								XYZ(0) += step(0);
								tMax(0) += tDelta(0);
							} else if (tMax(2) < tMax(0)) {
								XYZ(2) += step(2);
								tMax(2) += tDelta(2);
							// If 2 are equal minimum -> prioritize smallest tDelta between the two
							} else {
								XYZ(tDeltaMinDual(2)) += step(tDeltaMinDual(2));
								tMax(tDeltaMinDual(2)) += tDelta(tDeltaMinDual(2));
							}
						} else if (tMax(1) < tMax(0)) {
							// Normal
							if (tMax(1) < tMax(2)) {
								XYZ(1) += step(1);
								tMax(1) += tDelta(1);
							} else if (tMax(2) < tMax(1)) {
								XYZ(2) += step(2);
								tMax(2) += tDelta(2);
							// If 2 are equal minimum -> prioritize smallest tDelta between the two
							} else {
								XYZ(tDeltaMinDual(1)) += step(tDeltaMinDual(1));
								tMax(tDeltaMinDual(1)) += tDelta(tDeltaMinDual(1));
							}
						// Normal
						} else if (tMax(2) < tMax(0)) {
							XYZ(2) += step(2);
							tMax(2) += tDelta(2);
						// If 2 are equal minimum -> prioritize smallest tDelta between the two
						} else {
							XYZ(tDeltaMinDual(0)) += step(tDeltaMinDual(0));
							tMax(tDeltaMinDual(0)) += tDelta(tDeltaMinDual(0));
						}
					}
					if (blockedXYZ.find(XYZ) != blockedXYZ.end())
						return false;
				}
			}
		} else if (vi.count() == 2) {
			// Traverse between faces
			Vector2i axes;
			int ax;
			for (int i = 0; i < 3; i++)
				if (vi(i) == 0) {
					axes = Vector2i((i + 1) % 3, (i + 2) % 3);
					ax = i;
				}
			// Define start & end
			// Store face of higher-valued voxel
			Vector3i startFace(xyzEnd + (Vector3i(-1, -1, -1) + step) / 2);
			Vector3i endFace(xyzStart + (Vector3i(-1, -1, -1) - step) / 2);
			startFace(ax) = xyzEnd(ax);
			endFace(ax) = xyzStart(ax);
			Vector3f tDelta(0, 0, 0);
			tDelta(ax) = numeric_limits<float>::max();
			for (int i = 0; i < 2; i++)
				tDelta(axes(i)) = (float)step(axes(i)) / vi(axes(i));
			Vector3f tMax = tDelta; // starting from vertex, not crossing any boundaries

			// Check for corner moves
			bool doubleMove = (vi(axes(0)) == vi(axes(1)) &&
								startFace(axes(0)) != endFace(axes(0)) &&
								startFace(axes(1)) != endFace(axes(1)));
			for (int i = 0; i < 3; i++)
				if (startFace(i) == endFace(i)) tMax(i) = numeric_limits<float>::max();
			
			// Check for smallest tDelta
			int tDeltaMin = 0;
			float min_tDelta = tDelta.minCoeff(&tDeltaMin);

			Vector3i extra(0, 0, 0);
			extra(ax) = 1;
			vector<Vector3i> XYZ{startFace, startFace - extra};
			if (blockedXYZ.find(XYZ[0]) != blockedXYZ.end() && 
				blockedXYZ.find(XYZ[1]) != blockedXYZ.end())
				return false;
			
			if (doubleMove) {
				// cout << "doublemove 2 " << XYZ[0](0) << " " << XYZ[0](1) << " " << XYZ[0](2) << " " << endFace(0) << " " << endFace(1) << " " << endFace(2) << endl;
				while (XYZ[0] != endFace) {
					for (int i = 0; i < 2; i++) {
						XYZ[i] += step;
					}
					if (blockedXYZ.find(XYZ[0]) != blockedXYZ.end() && 
						blockedXYZ.find(XYZ[1]) != blockedXYZ.end())
						return false;
				}
			} else {
				// cout << "normalmove 2 " << XYZ[0](0) << " " << XYZ[0](1) << " " << XYZ[0](2) << " " << endFace(0) << " " << endFace(1) << " " << endFace(2) << endl;
				int tMaxMin = 0;
				float min_tMax = 0;
				while (XYZ[0] != endFace) {
					// if (XYZ[0](0) > -100 && XYZ[0](1) > -100)
					// cout << "\t " << tMax[0] << tMax[1] << XYZ[0](0) << " " << XYZ[0](1) << " " << XYZ[0](2) << " " << endl;
					// If both tMax are equal -> prioritize smallest tDelta
					min_tMax = tMax.minCoeff(&tMaxMin);
					if ((tMaxMin == axes(0) && tMax(axes(1)) == min_tMax) ||
						(tMaxMin == axes(1) && tMax(axes(0)) == min_tMax)) {
						for (int i = 0; i < 2; i++)
							XYZ[i](tDeltaMin) += step(tDeltaMin);
						tMax(tDeltaMin) += tDelta(tDeltaMin);
					// Else this is normal
					} else {
						for (int i = 0; i < 2; i++)
							XYZ[i](tMaxMin) += step(tMaxMin);
						tMax(tMaxMin) += tDelta(tMaxMin);
					}
					if (blockedXYZ.find(XYZ[0]) != blockedXYZ.end() && 
						blockedXYZ.find(XYZ[1]) != blockedXYZ.end())
						return false;
				}
			}
		} else if (vi.count() == 1) {
			// Traverse on axis
			Vector2i axes;
			int ax;
			for (int i = 0; i < 3; i++)
				if (vi(i) != 0) {
					axes = Vector2i((i + 1) % 3, (i + 2) % 3);
					ax = i;
				}
			// Define start & end
			// Store edge of higher-valued voxel
			Vector3i startEdge(xyzEnd);
			Vector3i endEdge(xyzStart);
			startEdge(ax) +=  (-1 + step(ax)) / 2;
			endEdge(ax) += (-1 - step(ax)) / 2;
			vector<Vector3i> extra{Vector3i(0, 0, 0), Vector3i(0, 0, 0), Vector3i(0, 0, 0), Vector3i(0, 0, 0)};
			extra[1](axes(0)) = 1;
			extra[2](axes(1)) = 1;
			extra[3](axes(0)) = 1;
			extra[3](axes(1)) = 1;
			vector<Vector3i> XYZ{startEdge, startEdge, startEdge, startEdge};
			for (int i = 0; i < 4; i++)
				XYZ[i] -= extra[i];
			if (blockedXYZ.find(XYZ[0]) != blockedXYZ.end() && 
				blockedXYZ.find(XYZ[1]) != blockedXYZ.end() && 
				blockedXYZ.find(XYZ[2]) != blockedXYZ.end() && 
				blockedXYZ.find(XYZ[3]) != blockedXYZ.end())
				return false;

			// cout << "normalmove 3 " << XYZ[0](0) << " " << XYZ[0](1) << " " << XYZ[0](2) << " " << endEdge(0) << " " << endEdge(1) << " " << endEdge(2) << endl;
			while (XYZ[0] != endEdge) {
				for (int i = 0; i < 4; i++)
					XYZ[i] += step;
				
				if (blockedXYZ.find(XYZ[0]) != blockedXYZ.end() && 
					blockedXYZ.find(XYZ[1]) != blockedXYZ.end() && 
					blockedXYZ.find(XYZ[2]) != blockedXYZ.end() && 
					blockedXYZ.find(XYZ[3]) != blockedXYZ.end())
					return false;
			}
		}
		return true;
	}

	// Set vertex function
	void SetVertex(Node& s) {
		if (s.xyz == s_start.xyz) return;
		if (!LineOfSight(s.parent->xyz, s.xyz)) {
			// bool yes = false;
			// if (s.xyz == Vector3i(39, 9, 17)) yes = true;
			// if (yes) cout << "parent " << s.parent->xyz(0) << " " << s.parent->xyz(1) << " " << s.parent->xyz(2) << " " << endl;
			auto nbs = GetVisNeighborsXYZ(s.xyz);
			float currentGCMin = numeric_limits<float>::max();
			// Vector3i currentXYZ;
			Vec3iu_map_i currentSearch, search;
			bool found = false;
			for (pair<Vector3i, float> i: nbs) {
				// cout << "neighbors 1 " << i.first(0) << " " << i.first(1) << " " << i.first(2) << " " << endl;
				search = closed.find(i.first);
				if (search != closed.end()) {
					found = true;
					if ((search->second)->gScore + i.second < currentGCMin) {
						currentGCMin = (search->second)->gScore + i.second;
						currentSearch = search;
						// currentXYZ = i.first;
					}
				}
			}
			if (found) {
				// cout << "new parent " << currentSearch->second->xyz(0) << " " << currentSearch->second->xyz(1) << " " << currentSearch->second->xyz(2) << " " << endl;
				s.parent = currentSearch->second;
				s.UpdateGScore(currentGCMin);
			}
		}
	}

	// Update vertex function
	void UpdateVertex(Node& s, Node& s_apos) {
		Vector3i dummy = s.parent->xyz;
		float cScore = sqrt((s_apos.xyz - dummy).squaredNorm());
		if (s.parent->gScore + cScore < s_apos.gScore) {
			Vec3iu_map_i search = open.find(s_apos.xyz);
			if (search != open.end()) {
				open.erase(s_apos.xyz);
				openList.erase(&s_apos);
			}
			s_apos.parent = s.parent;
			s_apos.UpdateGScore(s.parent->gScore + cScore);
			
			open.insert(make_pair(s_apos.xyz, &s_apos));
			openList.insert(&s_apos);
		}
	}

	// Implementing LazyTheta*-P algorithm
    // http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf
    // Params: start and end nodes
	bool ComputePath(const Vector3i& xyzStart, const Vector3i& xyzEnd) {
		// Start & end nodes
		s_end = Node(xyzEnd);
		s_start = Node(xyzStart, 0, &s_end);
		s_start.parent = &s_start;
		// If direct line of sight between start and end: return
		if (LineOfSight(s_start.xyz, s_end.xyz)) {
			s_end.parent = &s_start;
			return true;
		}

		nodes = vector<unique_ptr<Node>>{};
		open = Vec3iu_map{};
		openList = Nodemlt_set{};
		// openXYZ = Vec3iu_set{};
		closed = Vec3iu_map{};
		// closedXYZ = Vec3iu_set{};
		
		open.insert(make_pair(s_start.xyz, &s_start));
		openList.insert(&s_start);
		while (!open.empty()) {
			Node* s = *(openList.begin());
			// cout << "exploration " << s->xyz(0) << " " << s->xyz(1) << " " << s->xyz(2) << " " << endl;
			// cout << "explorer parent " << s->parent->xyz(0) << " " << s->parent->xyz(1) << " " << s->parent->xyz(2) << " " << endl;
			open.erase(s->xyz);
			openList.erase(s);
			// bool yes = false;
			// if (s->xyz == Vector3i(39, 9, 17)) yes = true;
			// if (yes) cout << "parent " << s->parent->xyz(0) << " " << s->parent->xyz(1) << " " << s->parent->xyz(2) << " " << endl;
			SetVertex(*s);
			// if (yes) cout << "parent " << s->parent->xyz(0) << " " << s->parent->xyz(1) << " " << s->parent->xyz(2) << " " << endl;
			// cout << "exploration " << s->xyz(0) << " " << s->xyz(1) << " " << s->xyz(2) << " " << endl;
			if (s->xyz == s_end.xyz) {
				// Path found
				s_end.parent = s->parent;
				return true;
			}

			closed.insert(make_pair(s->xyz, s));
			// Check 26 neighbors
			auto nbs = GetVisNeighborsXYZ(s->xyz);
			for (pair<Vector3i, float> i: nbs) {
				// cout << "exploration " << s->xyz(0) << " " << s->xyz(1) << " " << s->xyz(2) << " " << endl;
				// cout << "neighbors 2 " << i.first(0) << " " << i.first(1) << " " << i.first(2) << " " << endl;
				if (closed.find(i.first) == closed.end()) {
					Vec3iu_map_i search = open.find(i.first);
					Node* s_apos;
					if (search == open.end()) {
						nodes.push_back(unique_ptr<Node>(new Node(i.first, &s_end)));
						s_apos = (nodes.end() - 1)->get();
					} else {
						s_apos = search->second;
					}
					UpdateVertex(*s, *s_apos);
				}
			}
		}
		// Can't find path
		return false;
	}
};
