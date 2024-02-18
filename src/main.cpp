#include "gladiator.h"
#include <algorithm>
#include <chrono>
#include <cmath>

#undef abs

#define SIZE 12
#define DFS_DEPTH 9
#define M_TAU 6.283185307179586
#define EXPONENTIONAL_EXPONENT 0.9
#define MAC_0 "00:00:00:00:00:00"

constexpr int TEAM_ROBOTS = 2;
constexpr int TOTAL_ROBOTS = TEAM_ROBOTS << 1;

float EXPONENTS[DFS_DEPTH] = {1.0};

typedef enum e_dir { NORTH = 0, EAST, SOUTH, WEST, ROCKET } t_dir;
typedef enum e_momentum { FORWARD = 0, BACKWARD, STATIC, SIDE } t_momentum;

Gladiator* gladiator;
bool maze[SIZE][SIZE][4];
bool rockets[SIZE][SIZE];
bool connectedToCenter[SIZE][SIZE];
uint8_t possessions[SIZE][SIZE];
int stack[DFS_DEPTH][2];
uint8_t teamId = 0;
std::chrono::high_resolution_clock::time_point startTime;
float squareSize = 0.25f;
int frameCount = 0;
int currentTimeBlock = 0;
int robotsAlive = TOTAL_ROBOTS;
int opponentsAlive = TEAM_ROBOTS;
int minIdx = 0;
int maxIdx = SIZE - 1;
int goalX = SIZE >> 1;
int goalY = SIZE >> 1;
Position goalPos{0.0, 0.0, 0.0};
bool nextIsRocket = false;

bool reverseMode = false;

// Function to check if the robot is in a cul-de-sac
bool isInCulDeSac(const Position& pos) {
    int x = (int)(pos.x / squareSize);
    int y = (int)(pos.y / squareSize);
    int wallCount = 0;

    // Increase wallCount for each direction where there is a wall
    if (!maze[y][x][NORTH] || y == maxIdx) wallCount++;
    if (!maze[y][x][EAST] || x == maxIdx) wallCount++;
    if (!maze[y][x][SOUTH] || y == minIdx) wallCount++;
    if (!maze[y][x][WEST] || x == minIdx) wallCount++;

    return wallCount >= 3;
}

double reduceAngle(double x) {
	x = fmod(x + M_PI, M_TAU);
	if (x < 0) x += M_TAU;
	return x - M_PI;
}

double calculateDistance(const Position& from, const Position& to) {
	double dx = to.x - from.x;
	double dy = to.y - from.y;
	return std::sqrt(dx * dx + dy * dy);
}

double calculateAngleToTarget(const Position& from, const Position& to) {
	return atan2(to.y - from.y, to.x - from.x);
}

bool isTargetBehind(const Position& fromPos, const Position& toPos) {
    double angleToTarget = calculateAngleToTarget(fromPos, toPos);
    double angleDiff = reduceAngle(fromPos.a - angleToTarget);

    // Check if the target is behind within a certain angular threshold
    return fabs(angleDiff) > M_PI / 2 && fabs(angleDiff) < 3 * M_PI / 2;
}

double clamp(double x, double mini, double maxi) { return x < mini ? mini : x > maxi ? maxi : x; }

bool isSpeedLimited() { return gladiator->robot->getData().speedLimit < 0.3; }

bool shouldLaunchRocket() { return gladiator->weapon->canLaunchRocket() && opponentsAlive > 0; }

bool isRealRobotData(const RobotData& robotData) { return robotData.macAddress != MAC_0; }

bool willHit(const Position& myPos, const Position& enemyPos) {
	double maxRange = 4.5 * squareSize;
	double distanceToEnemy = calculateDistance(myPos, enemyPos);
	if (distanceToEnemy > maxRange) return false;
	double angleToEnemy = calculateAngleToTarget(myPos, enemyPos);
	double angleDifference = reduceAngle(myPos.a - angleToEnemy);
	return std::fabs(angleDifference) <= 0.15 / distanceToEnemy;
}

// void goTo(const Position& fromPos, const Position& toPos) {
// 	static const float wlimit = 0.4;
// 	static const float vlimit = 0.7;
// 	static const float epsilon = 0.07;
// 	double consvl, consvr;
// 	double dx = toPos.x - fromPos.x;
// 	double dy = toPos.y - fromPos.y;
// 	double d = sqrt(dx * dx + dy * dy);

// 	if (d > epsilon) {
// 		double rho = atan2(dy, dx);
// 		double angle = reduceAngle(rho - fromPos.a);
// 		double consw = clamp(angle, -wlimit, wlimit);
// 		double consv = clamp(clamp(d, 0.33, 1.5) * std::max(0.0, cos(angle)), -vlimit, vlimit);
// 		consvl = clamp(consv - consw, -1.0, 1.0);
// 		consvr = clamp(consv + consw, -1.0, 1.0);
// 		if (isSpeedLimited()) {
// 			double factor = gladiator->robot->getData().speedLimit /
// 							std::max(0.1, std::max(std::abs(consvl), std::abs(consvr)));
// 			consvl *= factor;
// 			consvr *= factor;
// 		}
// 	} else {
// 		consvl = 0.0;
// 		consvr = 0.0;
// 	}
// 	gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
// 	gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
// }

void goTo(const Position& fromPos, const Position& toPos, bool reverseMode = false) {
    static const float wlimit = 0.4;
    static const float vlimit = 0.7;
    static const float epsilon = 0.07;
    double consvl, consvr;
    double dx = toPos.x - fromPos.x;
    double dy = toPos.y - fromPos.y;
    double d = sqrt(dx * dx + dy * dy);


    reverseMode = isInCulDeSac(fromPos) && isTargetBehind(fromPos, toPos);
    if (d > epsilon) {
        double rho = atan2(dy, dx);

        // Adjust angle for reverse mode by adding PI (180 degrees) to invert direction
        double angle = reverseMode ? reduceAngle(rho - fromPos.a + M_PI) : reduceAngle(rho - fromPos.a);

        double consw = clamp(angle, -wlimit, wlimit);
        // For reverse mode, consider moving backwards by adjusting the calculation of consv
        double consv = reverseMode ? 
                       -clamp(d * cos(angle), -vlimit, vlimit) : // Invert speed for reverse
                       clamp(d * cos(angle), -vlimit, vlimit);

        consvl = clamp(consv - consw, -1.0, 1.0);
        consvr = clamp(consv + consw, -1.0, 1.0);

        if (isSpeedLimited()) {
            double factor = gladiator->robot->getData().speedLimit / std::max(0.1, std::max(std::abs(consvl), std::abs(consvr)));
            consvl *= factor;
            consvr *= factor;
        }
    } else {
        consvl = 0.0;
        consvr = 0.0;
    }

    // Apply the calculated wheel speeds
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
}



void dfsFromCenter(int y, int x) {
	if (x < minIdx || x > maxIdx || y < minIdx || y > maxIdx || connectedToCenter[y][x]) return;
	connectedToCenter[y][x] = true;
	if (maze[y][x][NORTH]) dfsFromCenter(y + 1, x);
	if (maze[y][x][EAST]) dfsFromCenter(y, x + 1);
	if (maze[y][x][SOUTH]) dfsFromCenter(y - 1, x);
	if (maze[y][x][WEST]) dfsFromCenter(y, x - 1);
}

void connectToCenter() {
	for (int y = 0; y < SIZE; ++y) {
		for (int x = 0; x < SIZE; ++x) {
			connectedToCenter[y][x] = false;
		}
	}
	dfsFromCenter(SIZE >> 1, SIZE >> 1);
}

void dfsGoal(size_t depth, float score, float* bestScore, bool isRocket) {
	if (score > *bestScore) {
		*bestScore = score;
		goalX = stack[1][0];
		goalY = stack[1][1];
		nextIsRocket = isRocket;
		if (!isSpeedLimited()) {
			int dx = stack[1][0] - stack[0][0];
			int dy = stack[1][1] - stack[0][1];
			for (size_t i = 2; i < depth; ++i) {
				if (stack[i][0] - stack[i - 1][0] != dx || stack[i][1] - stack[i - 1][1] != dy)
					break;
				goalX = stack[i][0];
				goalY = stack[i][1];
			}
		}
	}
	if (depth == DFS_DEPTH) return;
	float rocketValue = (shouldLaunchRocket() ? 1.2 : 3.0) * opponentsAlive;
	int prevDx, prevDy;
	if (depth == 1) {
		double angle = gladiator->robot->getData().position.a;
		// TODO: better calculation
		// TODO: inverse when going backwards
		
		double cosa = cos(angle);
		double sina = sin(angle);
		prevDx = cosa < -0.5 ? -1 : cosa > 0.5 ? 1 : 0;
		prevDy = sina < -0.5 ? -1 : sina > 0.5 ? 1 : 0;
	} else {
		prevDx = stack[depth - 1][0] - stack[depth - 2][0];
		prevDy = stack[depth - 1][1] - stack[depth - 2][1];
	}
	static const int neighbors[5][3] = {
		{0, 0, ROCKET}, {0, 1, NORTH}, {1, 0, EAST}, {0, -1, SOUTH}, {-1, 0, WEST},
	};
	for (size_t i = 0; i < 5; ++i) {
		t_dir dir = (t_dir)neighbors[i][2];
		if (depth != 1 && dir == ROCKET) continue;
		int x = stack[depth - 1][0] + neighbors[i][0];
		int y = stack[depth - 1][1] + neighbors[i][1];
		if (x < minIdx || x > maxIdx || y < minIdx || y > maxIdx) continue;
		float paintValue = possessions[y][x] == teamId ? 0.01 : possessions[y][x] == 0 ? 1 : 2;
		bool isWall = (dir == NORTH && !maze[y][x][SOUTH]) || (dir == EAST && !maze[y][x][WEST]) ||
					  (dir == SOUTH && !maze[y][x][NORTH]) || (dir == WEST && !maze[y][x][EAST]);
		if (isWall && (connectedToCenter[stack[depth - 1][1]][stack[depth - 1][0]] ||
					   !connectedToCenter[y][x]))
			continue;
		int newDx = x - stack[depth - 1][0];
		int newDy = y - stack[depth - 1][1];
		t_momentum momentum = (newDx == prevDx) && (newDy == prevDy)	 ? FORWARD
							  : (newDx == -prevDx) && (newDy == -prevDy) ? BACKWARD
							  : (dir == ROCKET)							 ? STATIC
																		 : SIDE;
		float momentumValue = momentum == FORWARD ? 1.0 : momentum == SIDE ? 0.5 : 0.0;
		float addScore = paintValue + rockets[y][x] * rocketValue + momentumValue;
		float newScore = score + addScore * EXPONENTS[depth];
		uint8_t prevPossession = possessions[y][x];
		bool prevRocket = rockets[y][x];
		possessions[y][x] = teamId;
		rockets[y][x] = false;
		stack[depth][0] = x;
		stack[depth][1] = y;
		dfsGoal(depth + 1, newScore, bestScore, depth == 1 ? prevRocket : isRocket);
		possessions[y][x] = prevPossession;
		rockets[y][x] = prevRocket;
	}
}

void updateGoal(const Position& myPosition) {
	int x = (int)(myPosition.x / squareSize);
	int y = (int)(myPosition.y / squareSize);
	goalX = SIZE >> 1;
	goalY = SIZE >> 1;
	stack[0][0] = x;
	stack[0][1] = y;
	float bestScore = 0.0;
	nextIsRocket = false;
	dfsGoal(1, 0.0, &bestScore, false);
	goalPos = {((float)goalX + 0.5f) * squareSize, ((float)goalY + 0.5f) * squareSize, 0.0};
}

void updateGlobals() {
	RobotList allBots = gladiator->game->getPlayingRobotsId();
	int newRobotsAlive = 0;
	opponentsAlive = 0;
	for (int i = 0; i < 4; i++) {
		RobotData other = gladiator->game->getOtherRobotData(allBots.ids[i]);
		if (!isRealRobotData(other)) continue;
		int y = (int)(other.position.y / squareSize);
		int x = (int)(other.position.x / squareSize);
		if (!other.lifes) {
			maze[y][x][NORTH] = false;
			maze[y][x][EAST] = false;
			maze[y][x][SOUTH] = false;
			maze[y][x][WEST] = false;
			if (y != maxIdx) maze[y + 1][x][SOUTH] = false;
			if (x != maxIdx) maze[y][x + 1][WEST] = false;
			if (y != minIdx) maze[y - 1][x][NORTH] = false;
			if (x != maxIdx) maze[y][x - 1][EAST] = false;
		} else {
			++newRobotsAlive;
			opponentsAlive += other.teamId != teamId;
			rockets[y][x] = false;
		}
	}
	if (robotsAlive != newRobotsAlive) {
		connectToCenter();
		robotsAlive = newRobotsAlive;
	}
	for (size_t y = 0; y < SIZE; ++y) {
		for (size_t x = 0; x < SIZE; ++x) {
			const MazeSquare* mazeSquare = gladiator->maze->getSquare(x, y);
			possessions[y][x] = mazeSquare->possession;
		}
	}
}

void tryLaunchingRockets(const RobotData& myRobot) {
	RobotList allBots = gladiator->game->getPlayingRobotsId();
	for (int i = 0; i < TOTAL_ROBOTS; i++) {
		RobotData other = gladiator->game->getOtherRobotData(allBots.ids[i]);
		if (isRealRobotData(other) && other.teamId != myRobot.teamId && other.lifes &&
			other.position.x > 0 && other.position.y > 0 && myRobot.position.x > 0 &&
			myRobot.position.y > 0 && willHit(myRobot.position, other.position)) {
			gladiator->weapon->launchRocket();
			return;
		}
	}
}

void checkTime() {
	auto currentTime = std::chrono::high_resolution_clock::now();
	auto deltaTime =
		std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
	int newTimeBlock = (deltaTime + 3000) / 20000;
	if (newTimeBlock != currentTimeBlock) {
		currentTimeBlock = newTimeBlock;
		++minIdx;
		--maxIdx;
		connectToCenter();
	}
}

void reset() {
	frameCount = 0;
	currentTimeBlock = 0;
	minIdx = 0;
	maxIdx = SIZE - 1;
	robotsAlive = TOTAL_ROBOTS;
	opponentsAlive = TEAM_ROBOTS;
	squareSize = gladiator->maze->getSquareSize();
	float middle = (SIZE - 1.0) * 0.5 * squareSize;
	goalPos = {middle, middle, 0.0};
	goalX = SIZE >> 1;
	goalY = SIZE >> 1;
	startTime = std::chrono::high_resolution_clock::now();
	teamId = gladiator->robot->getData().teamId;
	nextIsRocket = false;
	for (int y = 0; y < SIZE; ++y) {
		for (int x = 0; x < SIZE; ++x) {
			const MazeSquare* mazeSquare = gladiator->maze->getSquare(x, y);
			maze[y][x][NORTH] = !!mazeSquare->northSquare;
			maze[y][x][EAST] = !!mazeSquare->eastSquare;
			maze[y][x][SOUTH] = !!mazeSquare->southSquare;
			maze[y][x][WEST] = !!mazeSquare->westSquare;
			rockets[y][x] = mazeSquare->coin.value;
		}
	}
	connectToCenter();
}

void setup() {
	for (size_t i = 1; i < DFS_DEPTH; ++i)
		EXPONENTS[i] = EXPONENTS[i - 1] * EXPONENTIONAL_EXPONENT;
	gladiator = new Gladiator();
	gladiator->game->onReset(&reset);
}

void loop() {
	RobotData myRobot = gladiator->robot->getData();
	if (!gladiator->game->isStarted() || myRobot.lifes == 0) return;
	updateGlobals();
	Position myPosition = gladiator->robot->getData().position;
	if (frameCount == 0) delay(500);
	if ((frameCount & 63) == 0) checkTime();
	if (gladiator->weapon->canLaunchRocket() && opponentsAlive > 0) tryLaunchingRockets(myRobot);
	if ((frameCount & 15) == 0 ||
		(myPosition.x == goalX && myPosition.y == goalY && possessions[goalY][goalX] == teamId)) {
		updateGoal(myPosition);
		if (shouldLaunchRocket() && nextIsRocket) gladiator->weapon->launchRocket();
	}
	goTo(myPosition, goalPos);
	++frameCount;
	delay(5);
}