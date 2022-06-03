#include "lazytheta.cpp"
#include <vector>
#include <eigen3/Eigen/Dense>

void test2() {
	int size = 6;
	Vector3i OCC_SIZE_ZIP(150 / 2, 40 / 2, 30 / 2);
	vector<vector<Vector3i>> obst{
		{
			Vector3i(46, 4, 1),
			Vector3i(-62, 15, 9),
			Vector3i(24, -13, 0),
			Vector3i(63, 15, -5),
			Vector3i(-69, 9, -3),
			Vector3i(-6, -6, 14),
			Vector3i(-69, -13, 14),
			Vector3i(16, -13, 5),
			Vector3i(-64, 3, 5),
			Vector3i(51, -15, -4),
			Vector3i(68, 13, -15),
			Vector3i(29, 9, 3),
			Vector3i(63, -16, -2),
			Vector3i(1, 10, -6),
			Vector3i(-63, 13, -12),
			Vector3i(-16, -13, 3),
			Vector3i(6, -7, 10),
			Vector3i(-27, 12, 11),
			Vector3i(21, -16, 3)
		}, {
			Vector3i(-41, -18, -5),
			Vector3i(56, -15, -15),
			Vector3i(-44, 13, 14),
			Vector3i(-25, -17, 10),
			Vector3i(-50, 13, -10),
			Vector3i(13, -15, -9),
			Vector3i(27, 15, 4),
			Vector3i(17, -1, -5),
			Vector3i(65, -12, 7),
			Vector3i(57, -20, 1),
			Vector3i(57, -11, 1),
			Vector3i(57, -6, 0),
			Vector3i(-60, -15, 4),
			Vector3i(14, 3, -14),
			Vector3i(66, -10, -6),
			Vector3i(-52, -1, -5),
			Vector3i(-43, 3, -2),
			Vector3i(-64, 10, 6),
			Vector3i(53, -7, 5),
			Vector3i(43, 15, 5)
		}, {
			Vector3i(-32, -19, 11),
			Vector3i(68, 1, 3),
			Vector3i(55, 10, 10),
			Vector3i(-72, 10, -2),
			Vector3i(-60, 16, 12),
			Vector3i(3, -4, -5),
			Vector3i(43, -4, -15),
			Vector3i(-68, 4, -3),
			Vector3i(43, -8, -6),
			Vector3i(55, -12, -4),
			Vector3i(52, -12, -5),
			Vector3i(-51, -20, -3),
			Vector3i(18, -6, -2),
			Vector3i(-5, -20, -1),
			Vector3i(27, -2, -8),
			Vector3i(-64, -12, 4),
			Vector3i(-2, -16, -4),
			Vector3i(41, -18, -2),
			Vector3i(51, -19, 1)
		}
	};
	for (int n = 1; n < obst.size(); n++) {
		vector<Vector3i> obstSet;
		LazyTheta lt;
		cout << "done obstacle creation" << endl;
		for (int i = 0; i < obst[n].size(); i++) {
			for (int x = obst[n][i](0) - size; x <= obst[n][i](0) + size; x++)
				for (int y = obst[n][i](1) - size; y <= obst[n][i](1) + size; y++)
					for (int z = obst[n][i](2) - size; z <= obst[n][i](2) + size; z++) {
						// obstSet.push_back(Vector3i(x, y, z));
						lt.UpdateOccupancy(Vector3i(x, y, z));
					}
		}
		// lt.UpdateOccupancySet(obstSet);
		cout << "done obstacle expansion" << endl;
		auto start = -OCC_SIZE_ZIP;
		auto end = OCC_SIZE_ZIP;

		// Checking errors
		// if (n == 1 && !lt.LineOfSight(Vector3i(39, 9, 17), Vector3i(27, 5, 15))) {
		// 	cout << "ERR :::\n";
		// }
		Vector3i old_xyz;

		if (lt.ComputePath(start, end)) {
			Node& s = lt.s_end;
			// cout << "path " << s.xyz(0) << " " << s.xyz(1) << " " << s.xyz(2) << " " << endl;
			cout << "(" << s.xyz(0) << ", " << s.xyz(1) << ", " << s.xyz(2) << "), " << endl;
			while (s.xyz != start) {
				old_xyz = s.xyz;
				s = *s.parent;
				if (!lt.LineOfSight(old_xyz, s.xyz)) {
					cout << "ERR ::: "
					<< "(" << old_xyz(0) << ", " << old_xyz(1) << ", " << old_xyz(2) << "), "
					<< "(" << s.xyz(0) << ", " << s.xyz(1) << ", " << s.xyz(2) << "), " << endl;
				}
				cout << "(" << s.xyz(0) << ", " << s.xyz(1) << ", " << s.xyz(2) << "), " << endl;
			}
		}
	}
}


void testCpp() {
	int size = 3;
	Vector3i OCC_SIZE_ZIP(30 / 2, 20 / 2, 20 / 2);
	vector<vector<Vector3i>> obst{
		{Vector3i(1, -9, 7), Vector3i(9, 2, -7), Vector3i(0, 7, 7), Vector3i(-5, -10, 4), Vector3i(-3, -2, -3), Vector3i(14, 9, -10), Vector3i(14, -7, 9), Vector3i(-4, 5, -8), Vector3i(-4, 7, 6), Vector3i(-15, 3, 4), Vector3i(3, 7, -8), Vector3i(-2, -1, 5), Vector3i(-6, -9, 6), },
		{Vector3i(-12, 8, 6), Vector3i(10, -3, -7), Vector3i(3, 0, -4), Vector3i(10, -4, 6), Vector3i(11, 2, -8), Vector3i(3, -6, -9), Vector3i(-5, 6, -8), Vector3i(-15, -7, 6), Vector3i(14, -4, -9), Vector3i(10, -7, 6), Vector3i(-9, 2, 7), Vector3i(4, 1, -1), Vector3i(-5, 7, -6), Vector3i(7, 6, -10), },
		{Vector3i(1, 6, -6), Vector3i(1, 6, -3), Vector3i(0, 9, -9), Vector3i(7, -8, 9), Vector3i(-4, 1, -9), Vector3i(14, 7, -5), Vector3i(-5, -6, 1), Vector3i(-2, -9, 5), Vector3i(6, 6, -4), Vector3i(5, -5, -10), Vector3i(10, -10, 5), },
		{Vector3i(-2, 7, 9), Vector3i(6, -9, 8), Vector3i(12, -1, -2), Vector3i(-5, 4, -2), Vector3i(-3, 4, -4), Vector3i(-6, 6, -6), Vector3i(-4, 5, -6), Vector3i(-3, 1, -3), Vector3i(-2, -2, -10), Vector3i(-1, 2, -1), Vector3i(4, -1, -8), Vector3i(-4, 4, -9), },
		{Vector3i(-2, 5, -10), Vector3i(1, 1, -5), Vector3i(5, 7, -3), Vector3i(-8, -7, 5), Vector3i(-6, -2, -2), Vector3i(-8, 1, 2), Vector3i(-4, 2, 9), Vector3i(5, -2, 4), Vector3i(-3, 0, 9), Vector3i(-15, -4, 8), Vector3i(8, -5, -9), Vector3i(6, -2, 4), },
		{Vector3i(0, 9, -5), Vector3i(-3, 9, 9), Vector3i(-12, 2, 8), Vector3i(-15, 5, 3), Vector3i(-4, -6, 8), Vector3i(-6, -4, 7), Vector3i(14, -8, 6), Vector3i(11, -10, -7), Vector3i(-2, 1, -1), Vector3i(12, 1, -2), Vector3i(-3, 8, -5), Vector3i(-3, -5, 1), Vector3i(-1, 8, 6), },
		{Vector3i(-1, 6, -7), Vector3i(10, -8, -4), Vector3i(6, 0, 6), Vector3i(3, -4, -8), Vector3i(-6, -9, 6), Vector3i(-5, -1, 2), Vector3i(10, 5, -9), Vector3i(-9, 4, 7), Vector3i(-4, -8, 7), Vector3i(-10, 2, 5), Vector3i(7, -5, -8), },
		{Vector3i(2, -2, -7), Vector3i(0, -5, 1), Vector3i(-12, 5, -1), Vector3i(0, -4, 8), Vector3i(-2, -2, 8), Vector3i(-10, 3, -5), Vector3i(7, 6, -8), Vector3i(-1, 2, -3), Vector3i(-5, 5, 6), Vector3i(-15, -6, 9), Vector3i(5, -1, -1), },
		{Vector3i(-1, 3, 2), Vector3i(8, -5, 9), Vector3i(4, -10, -7), Vector3i(4, -10, -2), Vector3i(11, 6, -9), Vector3i(4, -2, 8), Vector3i(-4, 3, -4), Vector3i(-11, 6, -5), Vector3i(-3, 1, -7), Vector3i(3, -1, -1), Vector3i(-4, 4, 9), Vector3i(1, 7, 8), Vector3i(5, -7, -4), }
	};
	for (int n = 0; n < obst.size(); n++) {
		LazyTheta lt;
		cout << "done obstacle creation" << endl;
		vector<Vector3i> obstSet;
		for (int i = 0; i < obst[n].size(); i++) {
			for (int x = obst[n][i](0) - size; x <= obst[n][i](0) + size; x++)
				for (int y = obst[n][i](1) - size; y <= obst[n][i](1) + size; y++)
					for (int z = obst[n][i](2) - size; z <= obst[n][i](2) + size; z++) {
						// obstSet.push_back(Vector3i(x, y, z));
						lt.UpdateOccupancy(Vector3i(x, y, z));
					}
		}
		// lt.UpdateOccupancySet(obstSet);
		cout << "done obstacle expansion" << endl;
		auto start = -OCC_SIZE_ZIP;
		auto end = OCC_SIZE_ZIP;

		// Checking errors
		// if (n == 1 && !lt.LineOfSight(Vector3i(39, 9, 17), Vector3i(27, 5, 15))) {
		// 	cout << "ERR :::\n";
		// }
		Vector3i old_xyz;

		if (lt.ComputePath(start, end)) {
			Node& s = lt.s_end;
			// cout << "path " << s.xyz(0) << " " << s.xyz(1) << " " << s.xyz(2) << " " << endl;
			cout << "(" << s.xyz(0) << ", " << s.xyz(1) << ", " << s.xyz(2) << "), " << endl;
			while (s.xyz != start) {
				old_xyz = s.xyz;
				s = *s.parent;
				if (!lt.LineOfSight(old_xyz, s.xyz)) {
					cout << "ERR ::: "
					<< "(" << old_xyz(0) << ", " << old_xyz(1) << ", " << old_xyz(2) << "), "
					<< "(" << s.xyz(0) << ", " << s.xyz(1) << ", " << s.xyz(2) << "), " << endl;
				}
				cout << "(" << s.xyz(0) << ", " << s.xyz(1) << ", " << s.xyz(2) << "), " << endl;
			}
		}
	}
}

void testgate() {
	int OCC_SIZE_X = 150, OCC_SIZE_Y = 40, OCC_SIZE_Z = 30;
	Vector3i OCC_SIZE_ZIP(OCC_SIZE_X / 2, OCC_SIZE_Y / 2, OCC_SIZE_Z / 2);
	float OCC_UNIT_X = 0.1, OCC_UNIT_Y = 0.1, OCC_UNIT_Z = 0.1, OCC_UNIT = 0.1;
	LazyTheta lt;
	cout << "done obstacle creation" << endl;
    // Gate creation
    for (int z = 0; z < (int)round(1 / OCC_UNIT_Z); z++) {
		lt.UpdateOccupancy(Vector3i(int(round(4 / OCC_UNIT_X)), int(round(1 / OCC_UNIT_Y)), z) - OCC_SIZE_ZIP);
		lt.UpdateOccupancy(Vector3i(int(round(4 / OCC_UNIT_X)), int(round(3 / OCC_UNIT_Y)), z) - OCC_SIZE_ZIP);
	}
    for (int y = int(round(1 / OCC_UNIT_Y)); y < int(round(3 / OCC_UNIT_Y)) + 1; y++) {
		lt.UpdateOccupancy(Vector3i(int(round(4 / OCC_UNIT_X)), y, int(round(1 / OCC_UNIT_Z))) - OCC_SIZE_ZIP);
	}
    // Marker creation
    int X_CENTER= 14, Y_CENTER = 2;
    float RADIUS = 0.5;
    for (int x = int(round((X_CENTER - RADIUS) / OCC_UNIT_X)); x < int(round((X_CENTER + RADIUS) / OCC_UNIT_X + 1)); x++)
        for (int y = int(round((Y_CENTER - RADIUS) / OCC_UNIT_Y)); y < int(round((Y_CENTER + RADIUS) / OCC_UNIT_Y + 1)); y++)
			if ((Vector2f(X_CENTER / OCC_UNIT_X, Y_CENTER / OCC_UNIT_Y) - Vector2f(x, y)).norm() < RADIUS / OCC_UNIT)
				for (int z = 0; z < OCC_SIZE_Z; z += 6) {
					lt.UpdateOccupancy(Vector3i(x, y, z) - OCC_SIZE_ZIP);
					// cout << "obbie " << x << " " << y << " " << z << " " << endl;
				}

	cout << "done obstacle expansion" << endl;

    // Waypoints (can be dynamically defined later)
    Vector3i start	(- OCC_SIZE_X / 2, rand() % OCC_SIZE_Y - OCC_SIZE_Y / 2, - (rand() % (OCC_SIZE_Z / 2)));
    Vector3i end	(- OCC_SIZE_X / 2, rand() % OCC_SIZE_Y - OCC_SIZE_Y / 2, - (rand() % (OCC_SIZE_Z / 2)));
    vector<Vector3i> process = {
		start,
		Vector3i(-20, 4, -10),  // Past gate
		Vector3i(75, 4, -5),  // Left side of marker
		Vector3i(75, -4, -5),  // Left side of marker
		Vector3i(-20, -4, -10),  // Before gate
		end
	};
	vector<Vector3i> xyzs;
	Vector3i old_xyz;
	for (int i = 0; i < process.size() - 1; i++) {
		bool done = lt.ComputePath(process[i], process[i + 1]);
		if (done) {
			Node& s = lt.s_end;
			vector<Vector3i> proto_xyzs;
			proto_xyzs.push_back(s.xyz);
			// cout << "path " << s.xyz(0) << " " << s.xyz(1) << " " << s.xyz(2) << " " << endl;
			while (s.xyz != process[i]) {
				old_xyz = s.xyz;
				s = *s.parent;
				proto_xyzs.push_back(s.xyz);
			}
			reverse(proto_xyzs.begin(), proto_xyzs.end());
			xyzs.insert(xyzs.end(), proto_xyzs.begin(), proto_xyzs.end());
		}
		else cout << "ERROR::::::::::\n";
	}
	cout << "(" << xyzs[0](0) << ", " << xyzs[0](1) << ", " << xyzs[0](2) << "), "; // << endl;
	for (int i = 0; i < xyzs.size() - 1; i++) {
		if (!lt.LineOfSight(xyzs[i], xyzs[i + 1])) {
			cout << "ERR ::: "
			<< "(" << xyzs[i](0) << ", " << xyzs[i](1) << ", " << xyzs[i](2) << "), "
			<< "(" << xyzs[i + 1](0) << ", " << xyzs[i + 1](1) << ", " << xyzs[i + 1](2) << "), "
			<< endl;
		}
		if (!lt.LineOfSight(xyzs[i + 1], xyzs[i])) {
			cout << "ERR ::: "
			<< "(" << xyzs[i + 1](0) << ", " << xyzs[i + 1](1) << ", " << xyzs[i + 1](2) << "), "
			<< "(" << xyzs[i](0) << ", " << xyzs[i](1) << ", " << xyzs[i](2) << "), "
			<< endl;
		}
		cout << "(" << xyzs[i + 1](0) << ", " << xyzs[i + 1](1) << ", " << xyzs[i + 1](2) << "), "; // << endl;
	}
}

int main() {
	while (true)
	testgate();

	// Eigen::Matrix2d a;
	// a << 1, 2,
	// 	3, 4;
	// Eigen::MatrixXd b(2,2);
	// b << 2, 3,
	// 	1, 4;
	// std::cout << "a + b =\n" << a + b << std::endl;
	// std::cout << "a - b =\n" << a - b << std::endl;
	// std::cout << "Doing a += b;" << std::endl;
	// a += b;
	// std::cout << "Now a =\n" << a << std::endl;
	// Eigen::Vector3i v(1,2,3);
	// Eigen::Vector3i w(1,0,0);
	// std::cout << "-v + w - v =\n" << -v + w - v << std::endl;
	// while 
}