#include "gladiator.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#undef abs

// TODO: all pairs shortest path
// TODO: use previous wheel speed and update
// TODO: go backwards when appropriate
// TODO: detect when we have to go through walls
// TODO: shoot where people will be
// TODO: if next cell is rocket, shoot
// TODO: bezier/spline to predict better path
// TODO: find global variables to reset

#define SIZE 12
#define DFS_DEPTH 8
#define M_TAU 6.283185307179586
#define EXPONENTIONAL_EXPONENT 0.9

float EXPONENTS[DFS_DEPTH] = {1.0};

typedef enum e_dir { NORTH = 0, EAST, SOUTH, WEST, ROCKET } t_dir;

Gladiator* gladiator;
bool maze[SIZE][SIZE][4];
uint8_t teamId = 0;
std::chrono::high_resolution_clock::time_point startTime;
int frameCount = 0;
int currentTimeBlock = 0;
int minIdx = 0;
int maxIdx = SIZE - 1;
float squareSize = 0.0f;

int** stack;
uint8_t** possessions;
bool** connectedToCenter;

double reductionAngle(double x) {
	x = fmod(x + PI, 2 * PI);
	if (x < 0) x += 2 * PI;
	return x - PI;
}

double calculateDistance(const Position& myPos, const Position& enemyPos) {
	return std::sqrt(std::pow(enemyPos.x - myPos.x, 2) + std::pow(enemyPos.y - myPos.y, 2));
}

double normalizeAngle(double angle) {
	while (angle < 0)
		angle += 2 * M_PI;
	while (angle >= 2 * M_PI)
		angle -= 2 * M_PI;
	return angle;
}

double calculateAngleToTarget(const Position& from, const Position& to) {
	double dy = to.y - from.y;
	double dx = to.x - from.x;
	double angleToTarget = atan2(dy, dx);
	return normalizeAngle(angleToTarget);
}

double clamp(double x, double mini, double maxi) { return x < mini ? mini : x > maxi ? maxi : x; }

void goTo(Position fromPos, Position toPos) {
	static const float wlimit = 0.4;
	static const float vlimit = 0.8;
	static const float epsilon = 0.07;
	double consvl, consvr;
	double dx = toPos.x - fromPos.x;
	double dy = toPos.y - fromPos.y;
	double d = sqrt(dx * dx + dy * dy);

	if (d > epsilon) {
		double rho = atan2(dy, dx);
		double angle = reductionAngle(rho - fromPos.a);
		double consw = clamp(2 * angle, -wlimit, wlimit);
		double consv = clamp(clamp(d, 0.3, 3) * cos(angle), -vlimit, vlimit);
		consvl = clamp(consv - consw, -1.0, 1.0);
		consvr = clamp(consv + consw, -1.0, 1.0);
		double speedLimit = gladiator->robot->getData().speedLimit;
		if (speedLimit < 0.3) {
			double factor =
				speedLimit / std::max(0.1, std::max(std::abs(consvl), std::abs(consvr)));
			consvl *= factor;
			consvr *= factor;
		}
	} else {
		consvl = 0.0;
		consvr = 0.0;
	}
	gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
	gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
}

bool willHit(const Position& myPos, const Position& enemyPos, double myAngle) {
	double maxRange = 4 * squareSize;
	double distanceToEnemy = calculateDistance(myPos, enemyPos);

	if (distanceToEnemy > maxRange) return false;

	double myOrientation = normalizeAngle(myAngle);
	double angleToEnemy = calculateAngleToTarget(myPos, enemyPos);
	// Je suis tres sceptique de ces calculs avec les modulo 2x pi, ca explique sans doute des
	// oublis de tirs
	// Mettez tous les angles entre -pi et pi, et faites min(abs(x-y), 360+x-y, 360+y-x)

	double angleDifference = abs(myOrientation - angleToEnemy);
	angleDifference = std::min(angleDifference, 2 * M_PI - angleDifference);

	const double tolerance = M_PI / 24; // TODO: smaller tolerances for farther stuff

	return angleDifference <= tolerance;
}

void dfsFromCenter(int y, int x) {
	if (x < minIdx || x > maxIdx || y < minIdx || y > maxIdx || connectedToCenter[y][x]) return;
	connectedToCenter[y][x] = true;
	dfsFromCenter(y + 1, x);
	dfsFromCenter(y, x + 1);
	dfsFromCenter(y - 1, x);
	dfsFromCenter(y, x - 1);
}

void connectToCenter() {
	for (int y = 0; y < SIZE; ++y) {
		for (int x = 0; x < SIZE; ++x) {
			connectedToCenter[y][x] = false;
		}
	}
	dfsFromCenter(SIZE >> 1, SIZE >> 1);
}

void reset() {
	frameCount = 0;
	currentTimeBlock = 0;
	minIdx = 0;
	maxIdx = SIZE - 1;
	startTime = std::chrono::high_resolution_clock::now();
	squareSize = gladiator->maze->getSquareSize();
	teamId = gladiator->robot->getData().teamId;
	for (int y = 0; y < SIZE; ++y) {
		for (int x = 0; x < SIZE; ++x) {
			const MazeSquare* mazeSquare = gladiator->maze->getSquare(x, y);
			maze[y][x][NORTH] = !!mazeSquare->northSquare;
			maze[y][x][EAST] = !!mazeSquare->eastSquare;
			maze[y][x][SOUTH] = !!mazeSquare->southSquare;
			maze[y][x][WEST] = !!mazeSquare->westSquare;
		}
	}
}

void setup() {
	for (size_t i = 1; i < DFS_DEPTH; ++i) {
		EXPONENTS[i] = EXPONENTS[i - 1] * EXPONENTIONAL_EXPONENT;
	}
	stack = (int**)malloc(DFS_DEPTH * sizeof(int*));
	for (size_t i = 0; i < DFS_DEPTH; ++i)
		stack[i] = (int*)malloc(2 * sizeof(int));
	possessions = (uint8_t**)malloc(SIZE * sizeof(uint8_t*));
	connectedToCenter = (bool**)malloc(SIZE * sizeof(bool*));
	for (size_t i = 0; i < SIZE; ++i) {
		possessions[i] = (uint8_t*)malloc(SIZE * sizeof(uint8_t));
		connectedToCenter[i] = (bool*)malloc(SIZE * sizeof(bool));
	}
	gladiator = new Gladiator();
	gladiator->game->onReset(&reset);
}

void dfs(size_t depth, float score, float* bestScore, int* bestX, int* bestY) {
	static int neighbors[5][3] = {
		{0, 0, ROCKET}, {0, 1, NORTH}, {1, 0, EAST}, {0, -1, SOUTH}, {-1, 0, WEST},
	};
	if (score > *bestScore) {
		*bestScore = score;
		int dx = stack[1][0] - stack[0][0];
		int dy = stack[1][1] - stack[0][1];
		*bestX = stack[1][0];
		*bestY = stack[1][1];
		for (size_t i = 2; i < depth; ++i) {
			if (stack[i][0] - stack[i - 1][0] != dx || stack[i][1] - stack[i - 1][1] != dy) break;
			*bestX = stack[i][0];
			*bestY = stack[i][1];
		}
	}
	if (depth == DFS_DEPTH) return;
	float rocketValue = gladiator->weapon->canLaunchRocket() ? 2.0 : 5.0;
	for (size_t i = 0; i < 5; ++i) {
		t_dir dir = (t_dir)neighbors[i][2];
		if (depth != 1 && dir == ROCKET) continue;
		int x = stack[depth - 1][0] + neighbors[i][0];
		int y = stack[depth - 1][1] + neighbors[i][1];
		if (x < minIdx || x > maxIdx || y < minIdx || y > maxIdx) continue;
		float paintValue = possessions[y][x] == teamId ? 0.1 : possessions[y][x] == 0 ? 1 : 2;
		bool isWall = ((dir == NORTH && !maze[y][x][SOUTH]) || (dir == EAST && !maze[y][x][WEST]) ||
					   (dir == SOUTH && !maze[y][x][NORTH]) || (dir == WEST && !maze[y][x][EAST]));
		if (isWall && connectedToCenter[stack[depth - 1][1]][stack[depth - 1][0]]) continue;
		bool straightPath =
			(depth >= 2) &&
			((x - stack[depth - 1][0]) == (stack[depth - 1][0] - stack[depth - 2][0])) &&
			((y - stack[depth - 1][1]) == (stack[depth - 1][1] - stack[depth - 2][1]));
		bool goesBack = (depth >= 2) && (x == stack[depth - 2][0]) && (y == stack[depth - 2][1]);
		float addScore = paintValue + (possessions[y][x] & 4) * rocketValue -
						 0.2 * (dir == ROCKET) + 0.5 * straightPath - 0.5 * goesBack;
		float newScore = score + addScore * EXPONENTS[depth];
		uint8_t prev = possessions[y][x];
		possessions[y][x] = teamId;
		stack[depth][0] = x;
		stack[depth][1] = y;
		dfs(depth + 1, newScore, bestScore, bestX, bestY);
		possessions[y][x] = prev;
	}
}

void getNextMove(int x, int y, int* bestX, int* bestY) {
	for (size_t my = 0; my < SIZE; my++) {
		for (size_t mx = 0; mx < SIZE; mx++) {
			const MazeSquare* mazeSquare = gladiator->maze->getSquare(mx, my);
			possessions[my][mx] = mazeSquare->possession | mazeSquare->coin.value << 2;
		}
	}
	stack[0][0] = x;
	stack[0][1] = y;
	float bestScore = 0.0f;
	dfs(1, 0, &bestScore, bestX, bestY);
}

void loop() {
	if (gladiator->game->isStarted() && gladiator->robot->getData().lifes) {
		if (frameCount == 0) {
			delay(69); // TODO 500
		}
		++frameCount;
		if ((frameCount & 63) == 0) {
			auto currentTime = std::chrono::high_resolution_clock::now();
			auto deltaTime =
				std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime)
					.count();
			int newTimeBlock = (deltaTime + 2000) / 20000;
			if (newTimeBlock != currentTimeBlock) {
				currentTimeBlock = newTimeBlock;
				++minIdx;
				--maxIdx;
				connectToCenter();
			}
		}
		RobotData myRobot = gladiator->robot->getData();
		if (gladiator->weapon->canLaunchRocket()) {
			RobotList all_bots = gladiator->game->getPlayingRobotsId();
			for (int i = 0; i < 4; i++) {
				RobotData other = gladiator->game->getOtherRobotData(all_bots.ids[i]);
				if (other.teamId != myRobot.teamId && other.lifes && other.position.x > 0 &&
					other.position.y > 0 && myRobot.position.x > 0 && myRobot.position.y > 0) {
					if (willHit({myRobot.position.x, myRobot.position.y},
								{other.position.x, other.position.y}, {myRobot.position.a})) {
						gladiator->weapon->launchRocket();
						break;
					}
				}
			}
		}
		Position myPosition = gladiator->robot->getData().position;
		int x = (int)(myPosition.x / squareSize);
		int y = (int)(myPosition.y / squareSize);
		int bestX = SIZE >> 1;
		int bestY = SIZE >> 1;
		getNextMove(x, y, &bestX, &bestY);
		float targetX = ((float)bestX + 0.5f) * squareSize;
		float targetY = ((float)bestY + 0.5f) * squareSize;
		Position goal{targetX, targetY, 0};
		goTo(myPosition, goal);
		delay(5);
	}
}